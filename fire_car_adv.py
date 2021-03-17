import time
import threading, queue
import cv2
import numpy as np
import sys
from gpiozero import Robot, DistanceSensor, OutputDevice
from time import sleep
from threading import Thread

posX = 0
posY = 1700
hsvVal = [5,25,44,255,228,255]
global result

def target_reach(x,y,ar):
    
    print(x,y)
    
#    if ar > 400 and ar<700:
#        print("Move Forward")
#        robot.forward()
#        time.sleep(0.05)
#        robot.stop()
    
    if x>35:
        if y>15 or y<-7:   #-15
            print("Turn Right")
            robot.right()
            time.sleep(0.1)
            robot.stop()
            
    if x<-35:
        if y>15 or y<-3: #22,22/-15
            print("Turn Left")
            robot.left()
            time.sleep(0.1)
            robot.stop()
            
    if ar > 3100:
        if x<12 and y >-12:
            print("Start Pump")
        if x>-12 and y>-12:
            print("Start pump2")
        

    else:
        robot.stop()

def fire_capture():
    
    frameWidth = 240
    frameHeight = 160
    video = cv2.VideoCapture(0)
    video.set(3, frameWidth)
    video.set(4, frameHeight)
    

    while True:
        (grabbed, frame) = video.read()
        if not grabbed:
            break

        blur = cv2.GaussianBlur(frame, (21, 21), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        lower = [5, 44, 228]
        upper = [25, 255, 255]
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        mask = cv2.inRange(hsv, lower, upper)
        

        conts,h=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        for contoura in conts:
            
            peri = cv2.arcLength(contoura, True)
            approx = cv2.approxPolyDP(contoura, 0.02 * peri, True)
            #print(len(approx))
            
            
            area = cv2.contourArea(contoura)
            if(area > 500):
                x,y,w,h = cv2.boundingRect(contoura)
                
                c = max(conts, key=cv2.contourArea)
                #(x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                cv2.circle(frame, center, 3, (0, 0, 255), -1)
                
                
                cv2.rectangle(frame, (x,y),(x+w,y+h),(0,0,255),2)
                cv2.putText(frame, 'Fire', (x+w, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
                cv2.line(frame, (87, 71), center,(0,0,255),2)
                #print("mid:",center[0], center[1])
                diff_x = center[0]-87
                diff_y = center[1]-71
                print(area)
                
                target_reach(diff_x ,diff_y,area)
                

        cv2.imshow("Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    video.release()


def setup():
    print("Starting the Robotics...")
    global robot, sensor, relay
    robot = Robot(left = (27, 17), right = (22, 23))
    sensor = DistanceSensor(trigger=18, echo=24)
    relay = OutputDevice(25, active_high=True, initial_value=False)


			
def destroy():
    relay.off()
    robot.stop()
    cv2.destroyAllWindows()
    video.release()

class myThread (threading.Thread):
    def run(self):
        while True:
            sleep(2)
            distance = sensor.distance
            distance = sensor.distance * 100
            distance = round(sensor.distance, 2)
            print("Distance: {} cm".format(distance))
            if distance < 0.15:
                print("robot stopping")
                robot.stop()
                robot.backward()
                sleep(0.5)
                robot.stop()
            




if __name__ == '__main__':
	setup()
	thread1 = myThread()
	thread1.start()
	
	try:
            fire_capture()

	except KeyboardInterrupt:
            destroy()
