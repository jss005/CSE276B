'''
Created By: Jeremy Smith
Date: 2/6/2018

This file takes as input a word or series of words to search up on Wikipedia.
It gets the first sentence of the corresponding web page and saves it under
hernando/learned_objects/<input words>/description.txt

If the requested words did not have a web page, no file is created and a message
requesting a different description is given
'''

import re
import urllib2
import sys
import os
import shutil


if len(sys.argv) < 2:
  print "Error: need an object name to search"
  print "Usage: python googler.py 'filename containing word(s) to search'"
  sys.exit(-1)

#read in the object name 
obj_name_file = sys.argv[1]
fp = open(obj_name_file)
words = fp.readlines()[0]
fp.close()


#generate the url
words = re.sub(" +", '_', words)
url = "https://en.wikipedia.org/wiki/" + words

try:
  #grab the data from the web page
  response = urllib2.urlopen(url)
  html = response.read()

  #extract the first paragraph of the web page
  paragraphs = html.split("<p>")
  first_p = paragraphs[1]

  #strip out web-pagey stuff
  first_p = re.sub("<.*?>", '', first_p)
  first_p = re.sub("\[.*?\]", '', first_p)
  first_p = re.sub("\/.*?\/", '', first_p)

  #grab the first sentence
  sentence = first_p.split(".")[0]
  print sentence

  #write the sentence into the learned objects directory
  filename = "src/turtlebot_apps/hernando/learned_objects/" + words + "/description.txt"
  if not os.path.exists(os.path.dirname(filename)):
    os.makedirs(os.path.dirname(filename))
  fp = open(filename, 'w')
  fp.write(sentence)
  fp.close()

  #move images into the learned objects directory and remove .ini files
  images = [f for f in os.listdir("/home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/temp") \
            if os.path.isfile("/home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/temp/" + f)]
  for img in images:
    if img.endswith(".jpg"):
      shutil.move("/home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/temp/" + img,\
                  "/home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/learned_objects/" + words)
    elif img.endswith(".ini"):
      os.remove("/home/turtlebot/turtlebot_ws/src/turtlebot_apps/hernando/temp/" + img)
  

#error occurs when there is no webpage for the result
#likely occurs when a non-basic description is given
except:
  print "I couldn't find a definition for '%s'. Can you give me a more basic description?" % words
  sys.exit(-1)
