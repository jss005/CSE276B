'''
File: init_object_prompt.py
Author: Jeremy Smith
Date: 3/8/18

Opens a new terminal on the physical robot and asks the user to input the object's name
Note that the command "export DISPLAY=:0" must have been run beforehand through the ssh 
session
'''

import subprocess
import os

pid = subprocess.Popen(args=[
      "gnome-terminal", "--command=python src/turtlebot_apps/hernando/python_scripts/object_prompt.py"]).pid

os.waitpid(pid, 0)



