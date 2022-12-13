from math import ceil
from datetime import datetime

import pyqtgraph as pg
import time
import pyttsx3

from playsound import playsound

# global variables
lastSound = None
stepsPerMeter = 2.5
saveTextFile = 1  # 1 to log notifications to a text file. 0 otherwise


# change color of target based on centroid distance
# green - more than 4 meters away
# blue - more than 2 meters away
# yellow - between 1 and 2 meters away
# red - up to 1 meter away, +/- 0.6 meters to the left/right
def update_edge_color(x, x_spread, y):
    if 1 > abs(y) > 0.5:
        if abs(x) < 0.3:
            edge_color = pg.glColor('r')  # red
        elif abs(x) < 0.6:
            edge_color = pg.glColor('w')  # white
        else:
            edge_color = pg.glColor('g')  # green
    elif abs(y) < 2:
        if x < 0 and x+x_spread/2 > 0:
            edge_color = pg.glColor('y')  # yellow
        elif x > 0 and x-x_spread/2 < 0:
            edge_color = pg.glColor('y')  # yellow
        else:
            edge_color = pg.glColor('g')  # green
    elif abs(y) < 4 and abs(x) < 1:
        edge_color = pg.glColor('b')  # blue
    else:
        edge_color = pg.glColor('g')  # green
    return edge_color


def play_audio(audio, x, y):
    global lastSound

    # play audio if different from lastSound
    if audio != lastSound:
        playsound(audio + '.mp3')
        if audio != 'alert':  # allow alert to be repeated if necessary
            lastSound = audio

        if saveTextFile:
            fileName = 'binData/pHistText1.txt'
            tFile = open(fileName, 'a')
            tFile.write('time: ' + str(datetime.now()) + '(x, y): (' + str(x) + ', ' + str(y)
                        + ') ' + "sound: " + audio + '\n\n')
            tFile.close()
            print(audio + ' detection saved to file')


# Function will only be called when the x value of the object's centroid is within +/- 1 meter
def navigate(x, y, z, x_spread, edgeColor, ctext):
    location = '(' + str(x) + ',' + str(y) + ')'  # distance of obstacle from the sensor

    if edgeColor == pg.glColor('r'):  # distance = 1m, obstacle in path
        play_audio('alert', x, y)  # alert user of close obstacle
        text_speech = pyttsx3.init()
        text_speech.say('take 2 steps back')  # inform user to take two steps back
        text_speech.runAndWait()
        time.sleep(1)  # extra time to take two steps back
    elif edgeColor == pg.glColor('w'):  # distance = 1m, path is clear
        location = 'path clear'
        play_audio('bell', x, y)
    elif edgeColor == pg.glColor('y'):  # distance = 2m, give turn directions
        # create an object from the current obstacle dimensions
        ObjectSnapshot(x, y, z, x_spread, ctext)
    elif edgeColor == pg.glColor('b'):  # distance = 4m, obstacle approaching
        location = 'obstacle detected'
        play_audio('click', x, y)
    else:
        location = 'continue straight'

    ctext.setPosition(x, y, z, location)
    ctext.setVisible(True)


class ObjectSnapshot:
    def __init__(self, x, y, z, x_spread, ctext):
        # determine which side of the object is the closest
        # given the widths are equal and the sensor is always at coordinate 0:
        # -x means right edge is closer
        # x means left edge is closer
        self.text = 'right' if x < 0 else 'left'
        self.closest_edge = x - x_spread/2 if self.text == 'left' else x + x_spread/2
        self.run(x, y, z, ctext)

    # calculate steps it should take to go around an object of x width and set timeout to that time
    # if the object is still yellow repeat process with new dimension & location
    def run(self, x, y, z, ctext):
        global lastSound

        text_speech = pyttsx3.init()
        numSteps = ceil(abs(stepsPerMeter * self.closest_edge))
        if numSteps == 1:
            loc = "take " + str(numSteps) + " step " + str(self.text)
        else:
            loc = "take " + str(numSteps) + " steps " + str(self.text)

        ctext.setPosition(x, y, z, loc)
        ctext.setVisible(True)

        if saveTextFile:
            fileName = 'binData/pHistText1.txt'
            tFile = open(fileName, 'a')
            tFile.write('time: ' + str(datetime.now()) + '(x, y): (' + str(x) + ', ' + str(y)
                        + ') ' + "instruction: " + loc + '\n\n')
            tFile.close()
            print('yellow detection saved to file')

        text_speech.say(loc)
        text_speech.runAndWait()

        time.sleep(numSteps + 2)  # add 2 second buffer to the timer

        lastSound = None
