import enum
import re
import sys
import struct
from threading import Thread
import serial
from serial.tools.list_ports import comports
from . import bt

def pack(fmt, *args):
  return struct.pack('<' + fmt, *args)

def unpack(fmt, *args):
  return struct.unpack('<' + fmt, *args)

def multiord(b):
  if sys.version_info[0] >= 3:
    return list(b)
  else:
    return map(ord, b)

class Arm(enum.Enum):
  UNKNOWN = 0
  RIGHT = 1
  LEFT = 2

class XDirection(enum.Enum):
  UNKNOWN = 0
  X_TOWARD_WRIST = 1
  X_TOWARD_ELBOW = 2

class Pose(enum.Enum):
  REST = 0
  FIST = 1
  WAVE_IN = 2
  WAVE_OUT = 3
  FINGERS_SPREAD = 4
  THUMB_TO_PINKY = 5
  UNKNOWN = 255

class Myo(object):
  '''Implements the Myo-specific communication protocol.'''

  def __init__(self, tty=None):
    if tty is None:
      tty = self._detect_tty()
    if tty is None:
      raise ValueError('Myo dongle not found!')

    self.bt = bt.BT(tty)
    self.conn = None
    self.emg_handlers = []
    self.imu_handlers = []
    self.arm_handlers = []
    self.pose_handlers = []
    self.serial_exception_handlers = []
    self.running = False
    self.emg = [0,0,0,0,0,0,0,0]
    self.pose = Pose.UNKNOWN
    self.arm = Arm.UNKNOWN
    self.xdir = XDirection.UNKNOWN
    self.imu = (0,0,0)
    self.name = ""
    self.smooth_function = self.smooth
    self.smoothVar = [0,0,0,0,0,0,0,0]
    self.smoothed = [0,0,0,0,0,0,0,0]

  def _detect_tty(self):
    for p in comports():
      if re.search(r'PID=2458:0*1', p[2]):
        print('using device:', p[0])
        return p[0]

    return None

  def fetchData(self, timeout=1):
    res = self.bt.recv_packet(timeout)
    self.bt.flush()
    return res

  def run(self, timeout=1):
    self.connect()
    self.running = True
    while self.running:
      try:
        if self.fetchData(timeout) is None:
          self.disconnect()
          self.connect()
      except serial.serialutil.SerialException as e:
        self._on_serial_exception(e)

  def start(self, timeout=1):
    self.thread = Thread(target=self.run, args=(timeout,))
    self.thread.start()

  def stop(self):
    self.running = False
    self.disconnect()

  def connect(self):
    ## stop everything from before
    self.bt.end_scan()
    self.bt.disconnect(0)
    self.bt.disconnect(1)
    self.bt.disconnect(2)

    ## start scanning
    print('scanning...')
    self.bt.discover()
    while True:
      p = self.bt.recv_packet()
      print('scan response:', p)

      if p.payload.endswith(b'\x06\x42\x48\x12\x4A\x7F\x2C\x48\x47\xB9\xDE\x04\xA9\x01\x00\x06\xD5'):
        addr = list(multiord(p.payload[2:8]))
        break
    self.bt.end_scan()

    ## connect and wait for status event
    conn_pkt = self.bt.connect(addr)
    self.conn = multiord(conn_pkt.payload)[-1]
    self.bt.wait_event(3, 0)

    ## get firmware version
    fw = self.read_attr(0x17)
    _, _, _, _, v0, v1, v2, v3 = unpack('BHBBHHHH', fw.payload)
    print('firmware version: %d.%d.%d.%d' % (v0, v1, v2, v3))

    self.old = (v0 == 0)

    if self.old:
      ## don't know what these do; Myo Connect sends them, though we get data
      ## fine without them
      self.write_attr(0x19, b'\x01\x02\x00\x00')
      self.write_attr(0x2f, b'\x01\x00')
      self.write_attr(0x2c, b'\x01\x00')
      self.write_attr(0x32, b'\x01\x00')
      self.write_attr(0x35, b'\x01\x00')

      ## enable EMG data
      self.write_attr(0x28, b'\x01\x00')
      ## enable IMU data
      self.write_attr(0x1d, b'\x01\x00')

      ## Sampling rate of the underlying EMG sensor, capped to 1000. If it's
      ## less than 1000, emg_hz is correct. If it is greater, the actual
      ## framerate starts dropping inversely. Also, if this is much less than
      ## 1000, EMG data becomes slower to respond to changes. In conclusion,
      ## 1000 is probably a good value.
      C = 1000
      emg_hz = 50
      ## strength of low-pass filtering of EMG data
      emg_smooth = 100

      imu_hz = 50

      ## send sensor parameters, or we don't get any data
      self.write_attr(0x19, pack('BBBBHBBBBB', 2, 9, 2, 1, C, emg_smooth, C // emg_hz, imu_hz, 0, 0))

    else:
      self.name = self.read_attr(0x03).payload[5:]
      print('device name:', self.name)

      ## enable IMU data
      self.write_attr(0x1d, b'\x01\x00')
      ## enable on/off arm notifications
      self.write_attr(0x24, b'\x02\x00')

      # self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
      self._start_raw()

    ## add data handlers
    def handle_data(p):
      if (p.cls, p.cmd) != (4, 5):
        return

      _, attr, typ = unpack('BHB', p.payload[:4])
      pay = p.payload[5:]

      if attr == 0x27:
        vals = unpack('8HB', pay)
        ## not entirely sure what the last byte is, but it's a bitmask that
        ## seems to indicate which sensors think they're being moved around or
        ## something
        emg = vals[:8]
        moving = vals[8]
        self.emg = emg
        self._on_emg(emg, moving)
      elif attr == 0x1c:
        vals = unpack('10h', pay)
        quat = vals[:4]
        acc = vals[4:7]
        gyro = vals[7:10]
        self.imu = (quat, acc, gyro)
        self._on_imu(quat, acc, gyro)
      elif attr == 0x23:
        typ, val, xdir, _,_,_ = unpack('6B', pay)

        if typ == 1: # on arm
          self.arm, self.xdir = Arm(val), XDirection(xdir)
          self._on_arm(Arm(val), XDirection(xdir))
        elif typ == 2: # removed from arm
          self.arm, self.xdir = Arm.UNKNOWN, XDirection.UNKNOWN
          self._on_arm(Arm.UNKNOWN, XDirection.UNKNOWN)
        elif typ == 3: # pose
          self.pose = Pose(val)
          self._on_pose(Pose(val))
      else:
        print('data with unknown attr: %02X %s' % (attr, p))

    self.bt.add_handler(handle_data)

  def smooth(self, data, _):
    res = [0,0,0,0,0,0,0,0]
    for i, sensor in enumerate(data):
      self.smoothVar[i] = self.smoothVar[i] * 0.4 + sensor * 0.6
      res[i] = int(self.smoothVar[i] // 100)
    self.smoothed = res

  def write_attr(self, attr, val):
    if self.conn is not None:
      self.bt.write_attr(self.conn, attr, val)

  def read_attr(self, attr):
    if self.conn is not None:
      return self.bt.read_attr(self.conn, attr)
    return None

  def disconnect(self):
    if self.conn is not None:
      self.bt.disconnect(self.conn)

  def _start_raw(self):
    '''Sending this sequence for v1.0 firmware seems to enable both raw data and
    pose notifications.
    '''

    self.write_attr(0x28, b'\x01\x00')
    #self.write_attr(0x19, b'\x01\x03\x01\x01\x00')
    self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

  def mc_start_collection(self):
    '''Myo Connect sends this sequence (or a reordering) when starting data
    collection for v1.0 firmware; this enables raw data but disables arm and
    pose notifications.
    '''

    self.write_attr(0x28, b'\x01\x00')
    self.write_attr(0x1d, b'\x01\x00')
    self.write_attr(0x24, b'\x02\x00')
    self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
    self.write_attr(0x28, b'\x01\x00')
    self.write_attr(0x1d, b'\x01\x00')
    self.write_attr(0x19, b'\x09\x01\x01\x00\x00')
    self.write_attr(0x1d, b'\x01\x00')
    self.write_attr(0x19, b'\x01\x03\x00\x01\x00')
    self.write_attr(0x28, b'\x01\x00')
    self.write_attr(0x1d, b'\x01\x00')
    self.write_attr(0x19, b'\x01\x03\x01\x01\x00')

  def mc_end_collection(self):
    '''Myo Connect sends this sequence (or a reordering) when ending data collection
    for v1.0 firmware; this reenables arm and pose notifications, but
    doesn't disable raw data.
    '''

    self.write_attr(0x28, b'\x01\x00')
    self.write_attr(0x1d, b'\x01\x00')
    self.write_attr(0x24, b'\x02\x00')
    self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
    self.write_attr(0x19, b'\x09\x01\x00\x00\x00')
    self.write_attr(0x1d, b'\x01\x00')
    self.write_attr(0x24, b'\x02\x00')
    self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
    self.write_attr(0x28, b'\x01\x00')
    self.write_attr(0x1d, b'\x01\x00')
    self.write_attr(0x24, b'\x02\x00')
    self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

  def vibrate(self, length):
    if length in range(1, 4):
      ## first byte tells it to vibrate; purpose of second byte is unknown
      self.write_attr(0x19, pack('3B', 3, 1, length))

  def add_emg_handler(self, h):
    self.emg_handlers.append(h)

  def add_imu_handler(self, h):
    self.imu_handlers.append(h)

  def add_pose_handler(self, h):
    self.pose_handlers.append(h)

  def add_arm_handler(self, h):
    self.arm_handlers.append(h)

  def add_serial_exception_handler(self, h):
    self.serial_exception_handlers.append(h)

  def _on_emg(self, emg, moving):
    for h in self.emg_handlers:
      h(emg, moving)
    self.smooth_function(emg, moving)

  def _on_imu(self, quat, acc, gyro):
    for h in self.imu_handlers:
      h(quat, acc, gyro)

  def _on_pose(self, p):
    for h in self.pose_handlers:
      h(p)

  def _on_arm(self, arm, xdir):
    for h in self.arm_handlers:
      h(arm, xdir)

  def _on_serial_exception(self, e):
    for h in self.serial_exception_handlers:
      h(e)
