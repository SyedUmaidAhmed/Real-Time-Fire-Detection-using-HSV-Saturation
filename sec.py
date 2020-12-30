import cv2
import numpy as np
import matplotlib.pyplot as plt

def nothing(x):
    pass




cv2.namedWindow('Trac')
cv2.resizeWindow('Trac', 700, 512)

cv2.createTrackbar("hue min","Trac", 0, 179, nothing)
cv2.createTrackbar("hue max","Trac", 179, 179, nothing)
cv2.createTrackbar("sat min","Trac", 0, 255, nothing)
cv2.createTrackbar("sat max","Trac", 255, 255, nothing)
cv2.createTrackbar("val min","Trac", 0, 255, nothing)
cv2.createTrackbar("val max","Trac", 255, 255, nothing)


##img = cv2.imread('abc.png')
##hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
##cv2.imshow('win1', img)

cap = cv2.VideoCapture(0)

while True:
    _,frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("hue min","Trac")
    h_max = cv2.getTrackbarPos("hue max","Trac")
    s_min = cv2.getTrackbarPos("sat min","Trac")
    s_max = cv2.getTrackbarPos("sat max","Trac")
    val_min = cv2.getTrackbarPos("val min","Trac")
    val_max = cv2.getTrackbarPos("val max","Trac")
    #print(f'HUE MIN : {h_min} HUE MAX : SAT MIN : {s_min} SAT MAX : {s_max} VAL MIN : {val_min} VAL MAX : {val_max}')

    lower = np.array([h_min,s_min,val_min])
    upper = np.array([h_max,s_max,val_max])
    #mask = cv2.inRange(hsv_img, lower, upper)
    mask2 = cv2.inRange(hsv_frame, lower, upper)

    cv2.imshow('Real', frame)
    cv2.imshow('Mask', mask2)





    if cv2.waitKey(1) &0xFF == ord('q'):
        break