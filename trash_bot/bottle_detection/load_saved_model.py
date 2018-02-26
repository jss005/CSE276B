import pickle
from scipy.misc import imread
from sklearn import svm
import sys

def readAndGrayScale(filename):
  img = imread(filename)
  
  grayscale = []

  for i in range(0, len(img)):
    for j in range(0, len(img[0])):
      grayval = ((int)(img[i][j][0]) + (int)(img[i][j][1]) + (int)(img[i][j][2])) / 3 
      grayscale.append(grayval)

  return grayscale
    
#####MAIN#######

#get the input image, expected to be in the current directory as "screen.jpg"
img = readAndGrayScale("screen.jpg")

#load the saved bottle detector model
loaded_model = pickle.load(open("saved_bottle_detector.sav", 'rb'))

prediction = loaded_model.predict(img)

sys.exit(prediction)
