'''
recite_objects.py
Author: Jeremy Smith
Date: 3/11/2018

This file iterates through each directory in the learned_objects directory, displays the first image,
and recites the definition in description.txt
'''
import matplotlib.pyplot as plt
import subprocess
import sys 
import os
import time

obj_dir = "/home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/learned_objects/"

objects = os.listdir(obj_dir)

if len(objects) == 0:
  subprocess.call(["rosrun", "sound_play", "say.py",  "I don't know any objects yet"])

for obj in objects:
  #skip in case there is extra stuff in the directory
  if not os.path.isdir(obj_dir + obj):
    continue

  #get the first image and display it
  images = [f for f in os.listdir(obj_dir + obj) if f.endswith(".jpg")]
  img = plt.imread(obj_dir + obj + "/" + images[0])
  plt.imshow(img)
  plt.show(block=False)

  #recite the definition
  fp = open(obj_dir + obj + "/description.txt")
  description = fp.readlines()[0]
  fp.close()
  subprocess.call(["rosrun", "sound_play", "say.py",  description])

  sleep_time = 0.25 * len(description.split(' '))
  time.sleep(sleep_time)

  #close the image
  plt.close()
  
