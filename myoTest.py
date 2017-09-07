import sys
import time
import pyMyo.myo as myoLib

def arm(armData, xdir):
  print('Arm:', armData, 'Xdir:', xdir)

def pose(p):
  print("Pose:", p)

def main():
  time.sleep(2)
  while True:
    sys.stdout.write("\r" + str(myo.smoothed) + " " * 8)
    sys.stdout.flush()
    time.sleep(0.5)

if __name__ == '__main__':
  myo = myoLib.Myo()
  myo.add_arm_handler(arm)
  myo.add_pose_handler(pose)

  try:
    myo.start()
    main()
  except KeyboardInterrupt:
    pass
  finally:
    myo.stop()
