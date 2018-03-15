import time
import sys
from select import select

timeout = 30
print "Please type in the name of the object: "
rlist, _, _ = select([sys.stdin], [], [], timeout)

if rlist:
  name = raw_input("Please type in the name of the object: ")
  print "Thank you"
  time.sleep(1)
  filename = "src/turtlebot_apps/hernando/temp/obj_name.txt"
  fp = open(filename, 'w')
  fp.write(name)
  fp.close()

