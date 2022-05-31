
from ctypes import c_byte
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time




def setup():
    
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)              # GPIO Numbering
    GPIO.setup(motor_right_back,GPIO.OUT)  # All pins as Outputs
    GPIO.setup(motor_right_front,GPIO.OUT)  # All pins as Outputs
    GPIO.setup(motor_left_front,GPIO.OUT)  # All pins as Outputs
    GPIO.setup(motor_left_back,GPIO.OUT)  # All pins as Outputs
    
def forward():
    GPIO.output(motor_right_back,GPIO.LOW)
    GPIO.output(motor_right_front,GPIO.HIGH)
    GPIO.output(motor_left_back,GPIO.LOW)
    GPIO.output(motor_left_front,GPIO.HIGH)
def back():
    GPIO.output(motor_right_back,GPIO.HIGH)
    GPIO.output(motor_right_front,GPIO.LOW)
    GPIO.output(motor_left_back,GPIO.HIGH)
    GPIO.output(motor_left_front,GPIO.LOW)

def right():
        
    GPIO.output(motor_right_back,GPIO.HIGH)
    GPIO.output(motor_right_front,GPIO.LOW)
    GPIO.output(motor_left_back,GPIO.LOW)
    GPIO.output(motor_left_front,GPIO.HIGH)

def left():
    
    GPIO.output(motor_right_back,GPIO.LOW)
    GPIO.output(motor_right_front,GPIO.HIGH)
    GPIO.output(motor_left_back,GPIO.HIGH)
    GPIO.output(motor_left_front,GPIO.LOW)

def stop():
    GPIO.output(motor_right_back,GPIO.LOW)
    GPIO.output(motor_right_front,GPIO.LOW)
    GPIO.output(motor_left_back,GPIO.LOW)
    GPIO.output(motor_left_front,GPIO.LOW)

def get_stats(conts):
    c_x = 0
    c_y = 0
    area = 0
    x,y,w,h=cv2.boundingRect(conts)
    c_x = (x+x+w)/2
    c_y  = (y+y+h)/2
    area = w*h
    return c_x , c_y , area



motor_right_front = 32
motor_right_back = 36
motor_left_front = 38
motor_left_back = 40


#for red color
#lowerBound=np.array([170, 70,60])
#upperBound=np.array([180,255,255])

#for green color
lowerBound=np.array([33,80,40])
upperBound=np.array([102,255,255])

cam= cv2.VideoCapture(0)
kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))




setup()
#stop()
#left()
#time.sleep(0.5)
#right()
#time.sleep(0.5)
#stop()
#exit()

while True:
    ret, img=cam.read()
    img=cv2.resize(img,(320,220))
    mid_x  = 320/2
    mid_y = 220/2
    intial_area = 2000
    imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    # create the Mask
    mask=cv2.inRange(imgHSV,lowerBound,upperBound)
    #morphology
    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    maskFinal=maskClose
    conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    c_x = 0
    c_y =0
    area = 0
    if len(conts)!= 0:
        c_x , c_y , area = get_stats(conts[0])
        #print(c_x,c_y,area)
        if c_x > mid_x and abs(mid_x- c_x)>50:
            cv2.putText(img, "move right", (10, 10), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)
            print("right")
            right()
            time.sleep(0.05)
        elif c_x < mid_x and abs(mid_x- c_x)>50:
            cv2.putText(img, "move left", (10, 10), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)
            print("left")
            left()
            time.sleep(0.05)

        if area>intial_area and abs(intial_area - area)>2000:
            cv2.putText(img, "move back", (10, 10), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)
            print("backward")
            back()
            #time.sleep(0.1)
        
        elif area<intial_area and abs(intial_area - area)>1000:
            cv2.putText(img, "move front", (10, 10), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)
            print("forward")
            forward()
            #time.sleep(0.1)
        else:
            stop()
    
    else:
        stop()
    
    cv2.imshow("maskClose",maskClose)
    cv2.imshow("newthi",img)

    key = cv2.waitKey(30)
    if key == 27:
        break
    







