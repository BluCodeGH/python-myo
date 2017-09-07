import time
import json
import sys
import os
import traceback
import math
import pyMyo.myo as myoLib

#new dp, |sub from mean|, / std deviation
#std dev: sqrt((sumSq / n) - (sum / n)^2)

def identify1(emg, trainingData):
  best = ("Unknown", 0)
  for name, data in trainingData.items():
    matched = 0
    for i, bounds in enumerate(data):
      if bounds == [0,0] and emg[i] == 0:
        matched += 6.5
      if emg[i] >= bounds[0] and emg[i] <= bounds[1]:
        matched += (10 - (bounds[1] - bounds[0])) / 2
    if matched > best[1]:
      best = name, matched
  return best

def identify2(emg, trainingData):
  best = "rest", None
  for pose, data in trainingData.items():
    total = 0
    for i, val in enumerate(emg):
      dev = abs(data[i][0] - val)
      if data[i][1] == 0:
        total += dev * 1
      else:
        total += dev / data[i][1]
    if best[1] is None or total < best[1]:
      best = pose, total
  return best

def main2():
  with open("trainingData.json","r") as infile:
    trainingData = json.loads(infile.read())
  time.sleep(2)
  while True:
    name = input("Please enter the name of the gesture, 'q' to quit or 't' to test: ")
    if name == "q":
      break
    elif name =="t":
      test(trainingData)
      continue
    data = trainingData.get(name, None)
    if data is not None:
      res = None
      while res not in ["a", "r"]:
        res = input("This gesture already exists. Would you like to replace it or add to it? [r,a]: ")
      if res == "r":
        data = [[0.0,0.0] for i in range(8)]
      else:
        data = [[sensor[2], sensor[3]] for sensor in data]
    else:
      data = [[0.0,0.0] for i in range(8)]
    print("Starting recording in 2 seconds.")
    time.sleep(2)
    print("Recording started. Sampling at 5 hz. Press CRTL+C to stop.\n")
    n = 0
    while True:
      try:
        sys.stdout.write("\r" + str(myo.smoothed) + " " * 8)
        sys.stdout.flush()
        emg = myo.smoothed
        for i, val in enumerate(emg):
          data[i][0] += val
          data[i][1] += val * val
          if i == 0:
            print(val, val*val)
        n += 1
        time.sleep(0.2)
      except KeyboardInterrupt:
        break
    res = []
    for i, sensor in enumerate(data):
      mean = sensor[0] / n
      if sensor[1] / n - mean * mean < 0:
        print("ERR < ZERO", i)
        print(sensor)
        return
      stdDev = math.sqrt(sensor[1] / n - mean * mean)
      res.append([mean, stdDev, sensor[0], sensor[1]])
    print("Recording finished. Result:\n", name + ": " + str(res))
    trainingData[name] = res
    with open("trainingData.json","w+") as outfile:
      outfile.write(json.dumps(trainingData))

def main1():
  with open("trainingData.json","r") as infile:
    trainingData = json.loads(infile.read())
  time.sleep(2)
  while True:
    name = input("Please enter the name of the gesture, 'q' to quit or 't' to test: ")
    if name == "q":
      break
    elif name =="t":
      test(trainingData)
      continue
    data = trainingData.get(name, [])
    if data != []:
      res = None
      while res not in ["a", "r"]:
        res = input("This gesture already exists. Would you like to replace it or add to it? [r,a]: ")
      if res == "r":
        data = []
    print("Starting recording in 2 seconds.")
    time.sleep(2)
    print("Recording started. Sampling at 5 hz. Press any key to stop.\n")
    while True:
      try:
        sys.stdout.write("\r" + str(myo.smoothed) + " " * 8)
        sys.stdout.flush()
        #if kbPress():
          #break
        emg = myo.smoothed
        if data == []:
          for val in emg:
            data.append([val, val])
          continue
        for i, val in enumerate(emg):
          if val < data[i][0]:
            data[i][0] = val
          elif val > data[i][1]:
            data[i][1] = val
        time.sleep(0.2)
      except KeyboardInterrupt:
        break
    print("Recording finished. Result:\n", name + ": " + str(data))
    trainingData[name] = data
    with open("trainingData.json","w+") as outfile:
      outfile.write(json.dumps(trainingData))

def test(trainingData):
  print("Starting test, press any key to exit.")
  old = None
  while True:
    try:
      emg = myo.smoothed
      best = identify2(emg, trainingData)
      if old != best:
        print(best)
        old = best
      time.sleep(1)
    except KeyboardInterrupt:
      break

if __name__ == '__main__':
  myo = myoLib.Myo()
  try:
    myo.start()
    main2()
  except Exception as e:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    traceback.print_exception(exc_type, exc_value, exc_traceback, file=sys.stdout)
  myo.stop()
