import cv2
import numpy as np

#video_file = "video_1.mp4"
video = cv2.VideoCapture(0)

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

#    kernelOpen = np.ones((5,5))
#    kernelClose = np.ones((20,20))

    mask = cv2.inRange(hsv, lower, upper)

#    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
#    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
#    maskFinal = maskClose

    #ret, binary = cv2.threshold(mask,127,255,cv2.THRESH_BINARY_INV)


    conts,h=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for contoura in conts:
        area = cv2.contourArea(contoura)
        if(area > 400):
            x,y,w,h = cv2.boundingRect(contoura)
            cv2.rectangle(frame, (x,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(frame, 'Fire', (x+w, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)


    output = cv2.bitwise_and(frame, hsv, mask=mask)
    no_red = cv2.countNonZero(mask)
    cv2.imshow("output", output)

    if int(no_red) > 20000:
        print ('Fire detected')

    cv2.imshow("Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
video.release()