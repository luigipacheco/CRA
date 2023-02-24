__author__ = "Luis Pacheco"
__copyright__ = "Copyright 2019, Luis Pacheco"
__contributors__ = "Ricardo Mura"
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Luis Pacheco"
__email__ = "luigi@luigipacheco.com"
__status__ = "Alpha"

import os
import serial
import time
import cv2
import imutils
import math
import urllib
import numpy as np

from kivy.app import App
from kivy.uix.button import Button
from kivy.clock import Clock
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import  BoxLayout
from kivy.uix.textinput import TextInput
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
import numpy as np



step = 1
testing = True
low_blue = np.array([100,50,100])
high_blue = np.array([140,255,255])
low_red = np.array([20, 50, 70])
high_red = np.array([70, 255, 255])
camera_index = 0
tol = 20
class KivyCamera(Image):
    def __init__(self, capture, fps, **kwargs):
        super(KivyCamera, self).__init__(**kwargs)
        self.capture = capture
        Clock.schedule_interval(self.update, 1.0 / fps)
        self.xdist = 0
        self.ydist = 0
        self.gotoOn = False
        Clock.schedule_interval(self.goto, 0.5)


    def xDistance(self, x, x1):
        dx = x1 - x
        return dx

    def yDistance(self, y, Y1):
        dy = y1 - y
        return dy

    def calculateDistance(self, dx, dy):
        dist = math.sqrt(dx ** 2 + dy ** 2)
        return dist

    def findcolor(self, a, b, hsv, frame):
        low = np.array([a])
        high = np.array([b])
        mask = cv2.inRange(hsv, low, high)
        mask1 = cv2.bitwise_and(frame, frame, mask=mask)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        if len(cnts) < 1:
            return
        # cnts = imutils.grab_contours(cnts)
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] == 0 :  return
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
        cv2.circle(frame, center, 5, (255, 0, 0), -1, 8, 0)
        return center

    def update(self, dt):
        ret, frame = self.capture.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #cv2.line(frame, (0, 0), (511, 511), (255, 0, 0), 5)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Red color
        self.redcenter = self.findcolor(low_red, high_red, hsv_frame, frame)
        # Blue color
        self.bluecenter = self.findcolor(low_blue, high_blue,hsv_frame, frame)

        if self.redcenter and self.bluecenter:
            # self.xdist = self.xDistance(redcenter[0], bluecenter[0])
            # self.ydist = self.xDistance(redcenter[1], bluecenter[1])
            # absolutedistance = self.calculateDistance(xdist, ydist)
            # print(absolutedistance)
            self.xdist = self.xDistance(self.redcenter[0], self.bluecenter[0])
            self.ydist = self.xDistance(self.redcenter[1], self.bluecenter[1])



        if ret:
            # convert it to texture
            buf1 = cv2.flip(frame, 0)
            buf = buf1.tostring()
            image_texture = Texture.create(
                size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
            image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
            # display image from the texture
            self.texture = image_texture
    def goToTrigger(self):
        if self.gotoOn == False:
            self.gotoOn = True
        else:
            self.gotoOn = False

    def goto(self,dt):
        if self.gotoOn == True:
            if self.bluecenter and self.redcenter:
                if self.xdist <= (-tol):
                    moveX = 10
                    data = "G0 X{0:d}\n".format(moveX)
                    print(data)
                    if testing:
                        s.write(data.encode())

                elif self.xdist >= (+tol):
                    moveX = 10
                    data = "G0 X-{0:d}\n".format(moveX)
                    print(data)
                    if testing:
                        s.write(data.encode())

                if self.ydist <= (-tol):
                    moveY = 10
                    data = "G0 Y{0:d}\n".format(moveY)
                    print(data)
                    if testing:
                        s.write(data.encode())

                elif self.ydist >= (+tol):
                    moveY = 10
                    data = "G0 Y-{0:d}\n".format(moveY)
                    print(data)
                    if testing:
                        s.write(data.encode())
                # if self.xdist <= tol and self.ydist <= tol:
                #     self.goToTrigger()


class mainLayout(GridLayout):
    def __init__(self):
        super(mainLayout, self).__init__()
        self.cols = 2
        self.cameraOn = False

        if os.path.isfile("prev_settings.txt"):
            with open("prev_settings.txt","r") as f:
                d = f.read().split(",")
                prev_port = d[0]
                prev_baudrate = d[1]
                prev_width= d[2]
                prev_steps = d[3]
                prev_mmpr = d[4]
                prev_speed = d[5]
        else:
            prev_port = ""
            prev_baudrate = ""
            prev_width=""
            prev_steps = ""
            prev_mmpr = ""
            prev_speed = ""

        port = self.ids['port']
        baudrate = self.ids['baudrate']
        machineWidth = self.ids['machineWidth']
        steps = self.ids['setStep']
        mmpr = self.ids['mmPerMin']
        speed = self.ids['speed']

        port.text= prev_port
        baudrate.text = prev_baudrate
        machineWidth.text = prev_width
        steps.text = prev_steps
        mmpr.text=prev_mmpr
        speed.text = prev_speed

    def camera(self):
        layout = self.ids['opencv']
        self.capture = cv2.VideoCapture(camera_index)
        self.my_camera = KivyCamera(capture=self.capture, fps=30)
        layout.add_widget(self.my_camera)

    def goTo(self):
            self.my_camera.goToTrigger()

    def on_stop(self):
        # without this, app will not exit even if the window is closed
        self.capture.release()

    def save(self):
        port = self.ids['port'].text
        baudrate = self.ids['baudrate'].text
        machineWidth = self.ids['machineWidth'].text
        steps = self.ids['setStep'].text
        mmpr = self.ids['mmPerMin'].text
        speed = self.ids['speed'].text
        toolWidth = "3"
        print("cake has been pressed")
        self.sendSpecs(machineWidth,toolWidth,steps,float(mmpr)*3.14159)
        self.setHome(machineWidth)
        self.sendSpeed(speed)
        with open("prev_settings.txt","w") as f:
             f.write(f"{port},{baudrate},{machineWidth},{steps},{mmpr},{speed}")

    def connect(self):
        time.sleep(1)
        port = self.ids['port'].text
        baudrate = self.ids['baudrate'].text
        machineWidth = self.ids['machineWidth'].text
        steps = self.ids['setStep'].text
        mmpr = self.ids['mmPerMin'].text
        speed = self.ids['speed'].text
        toolWidth = "3"
        s = serial.Serial('/dev/'+port, int(baudrate))
        time.sleep(1)
        print(f"cake has been pressed")
        self.sendSpecs(machineWidth,toolWidth,steps,float(mmpr)*3.14159)
        self.setHome(machineWidth)
        self.sendSpeed(speed)
        print('Opening Serial Port')

    def send(self,data):
        print("output = " + data)
        if testing:
            s.write(data.encode())

    def setHome(self,machineWidth):
        if machineWidth == False:
            machineWidth = self.ids['machineWidth'].text
        data = "M1 X00 Y"+ str(int(machineWidth)/2)+ "\n"
        self.send(data)


    def sendRelative(self):
        msg = "G91\n"
        self.send(msg)

    def sendSpecs(self,machineWidth,penWidth,stepsPerRev,mmPerRev):
        specs = "M4 X" + str(machineWidth) + " E" + str(penWidth) + " S" + str(stepsPerRev) + " P" + str(
            mmPerRev) + "\n"
        self.send(specs)

    def sendSpeed(self, speed):
        speed = "G0 F" + str(speed) + "\n"
        self.send(speed)

    def moveXY(self):
        self.sendRelative()
        time.sleep(1)
        moveX = int(self.ids['moveX'].text)
        moveY = int(self.ids['moveY'].text)
        data = "G0 X{0:d} Y{1:d}\n".format(moveX, moveY)
        self.send(data)

    def moveUP(self):
        self.sendRelative()
        myid = self.ids['step']
        moveY = int(myid.text)
        data = "G0 Y-{0:d}\n".format(moveY)
        self.send(data)

    def moveRIGHT(self):
        self.sendRelative()
        myid = self.ids['step']
        moveX = int(myid.text)
        data = "G0 X{0:d}\n".format(moveX)
        self.send(data)

    def moveLEFT(self):
        self.sendRelative()
        myid = self.ids['step']
        moveX = int(myid.text)
        data = "G0 X-{0:d}\n".format(moveX)
        self.send(data)


    def moveDOWN(self):
        self.sendRelative()
        myid = self.ids['step']
        moveY = int(myid.text)
        data = "G0 Y{0:d}\n".format(moveY)
        self.send(data)

    def sendMotorsOff(self):
        data = "M84\n"
        self.send(data)

    def emergencyStop(self):
        data = "M112\n"
        self.send(data)

    def drawQuad(self):
        self.sendRelative()
        moveX = int(self.ids['quadX'].text)
        moveY = int(self.ids['quadY'].text)
        data = "G0 X{0:d}\n".format(moveX)
        self.send(data)
        data = "G0 Y{0:d}\n".format(moveY)
        self.send(data)
        data = "G0 X-{0:d}\n".format(moveX)
        self.send(data)
        data = "G0 Y-{0:d}\n".format(moveY)
        self.send(data)
    # def drawOnTarget(self):
    #     if self.my_camera.xdist >= xdist or self.my_camera.ydist >= tol:
    #         self.goTo()
    #         if


class guiApp(App):
    def build(self):
        self.button_size = 150
        return mainLayout()

if __name__ == "__main__":
    if testing:
        try:
            s = serial.Serial('/dev/ttyACM0', 115200)
            #s = Serial(port=port, baudrate=self.baudrate, timeout=self.timeout)
        except:
            print("Failed to connect")
    guiApp().run()
    cv2.destroyAllWindows()
