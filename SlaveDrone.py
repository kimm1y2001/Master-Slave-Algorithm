from djitellopy import Tello
import cv2
import numpy as np
from time import sleep

global imgContour
global startCounter
global direction, cap2
slave = Tello(host="192.168.31.100", vs_udp=11111)



# ------------------------------------------------------------------
WIDTH = 640  # WIDTH OF THE IMAGE
HEIGHT = 480  # HEIGHT OF THE IMAGE
DEADZONE = 120
startCounter = 0
direction = 0
# -------------------------------------------------------------------

frameWidth = WIDTH
frameHeight = HEIGHT

def empty(a):
    pass

def slaveInitialize():

    slave.connect()
    # slave.change_vs_udp(11113)
    slave.for_back_velocity = 0
    slave.left_right_velocity = 0
    slave.up_down_velocity = 0
    slave.yaw_velocity = 0
    slave.speed = 0

    print(slave.get_battery())

    slave.streamoff()
    slave.streamon()
    sleep(1)
    # cap2 = cv2.VideoCapture(2)


def createTrackbar():
    cv2.namedWindow("HSV")
    cv2.resizeWindow("HSV", 640, 240)
    cv2.createTrackbar("HUE Min", "HSV", 115, 179, empty)
    cv2.createTrackbar("HUE Max", "HSV", 134, 179, empty)
    cv2.createTrackbar("SAT Min", "HSV", 85, 255, empty)
    cv2.createTrackbar("SAT Max", "HSV", 199, 255, empty)
    cv2.createTrackbar("VALUE Min", "HSV", 0, 255, empty)
    cv2.createTrackbar("VALUE Max", "HSV", 174, 255, empty)

    cv2.namedWindow("Parameters")
    cv2.resizeWindow("Parameters", 640, 240)
    cv2.createTrackbar("Threshold1", "Parameters", 58, 255, empty)
    cv2.createTrackbar("Threshold2", "Parameters", 93, 255, empty)
    cv2.createTrackbar("Area", "Parameters", 3540, 30000,empty)


def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]

    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)

        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)

    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor

    return ver


def getContours(img,imgContour):
    global direction
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area", "Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            #print(len(approx))
            x , y , w, h = cv2.boundingRect(approx)
            cx = int(x + (w / 2))  # CENTER X OF THE OBJECT
            cy = int(y + (h / 2))  # CENTER Y OF THE OBJECT


            if (cx < int(frameWidth / 2) - DEADZONE * 0.8):
                cv2.putText(imgContour, " GO LEFT " , (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                cv2.rectangle(imgContour, (0,int(frameHeight / 2 - DEADZONE)), (int(frameWidth / 2) - DEADZONE, int(frameHeight / 2) + DEADZONE), (0, 0, 255), cv2.FILLED)
                direction = 1
                print(direction)
            elif (cx > int(frameWidth / 2) + DEADZONE * 0.8):
                cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 + DEADZONE), int(frameHeight / 2 - DEADZONE)), (frameWidth, int(frameHeight / 2) + DEADZONE), (0, 0, 255), cv2.FILLED)
                direction = 2
                print(direction)
            elif (cy < int(frameHeight / 2) - DEADZONE * 0.4):
                cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - DEADZONE), 0), (int(frameWidth / 2 + DEADZONE), int(frameHeight / 2) - DEADZONE), (0, 0, 255), cv2.FILLED)
                direction = 3
                print(direction)
            elif (cy > int(frameHeight / 2) + DEADZONE * 0.4):
                cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1,(0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - DEADZONE), int(frameHeight / 2) + DEADZONE), (int(frameWidth / 2 + DEADZONE), frameHeight), (0, 0, 255), cv2.FILLED)
                direction = 4
                print(direction)
            else:
                if (area > 12500):
                    cv2.putText(imgContour, " GO BACKWARDS ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    direction = -99
                elif (area < 8500):
                    cv2.putText(imgContour, " GO FORWARDS ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    direction = 99
                else:
                    direction = 0

            cv2.line(imgContour, (int(frameWidth/2),int(frameHeight/2)), (cx,cy),(0, 0, 255), 3)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,(0, 255, 0), 2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,(0, 255, 0), 2)
            cv2.putText(imgContour, " " + str(int(x)) + " " + str(int(y)), (x - 20, y - 45), cv2.FONT_HERSHEY_COMPLEX,0.7,(0, 255, 0), 2)

            break

        else: direction = 0


def display(img):
    # Creating the 9 region of detection
    cv2.line(img, (int(frameWidth/2) - DEADZONE, 0), (int(frameWidth / 2) - DEADZONE, frameHeight), (255, 255, 0), 3)
    cv2.line(img, (int(frameWidth/2) + DEADZONE, 0), (int(frameWidth / 2) + DEADZONE, frameHeight), (255, 255, 0), 3)
    cv2.circle(img,(int(frameWidth/2),int(frameHeight/2)),5,(0,0,255),5)
    cv2.line(img, (0, int(frameHeight / 2) - DEADZONE), (frameWidth, int(frameHeight / 2) - DEADZONE), (255, 255, 0), 3)
    cv2.line(img, (0, int(frameHeight / 2) + DEADZONE), (frameWidth, int(frameHeight / 2) + DEADZONE), (255, 255, 0), 3)

def runSlave():
    global startCounter
    global direction
    while True:

        # Tello image processing procedure into window display
        frame_read = slave.get_frame_read()
        myFrame = frame_read.frame
        ImageSlave = cv2.resize(myFrame, (WIDTH, HEIGHT))
        imgContour = ImageSlave.copy()
        imgHsv = cv2.cvtColor(ImageSlave, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("HUE Min","HSV")
        h_max = cv2.getTrackbarPos("HUE Max", "HSV")
        s_min = cv2.getTrackbarPos("SAT Min", "HSV")
        s_max = cv2.getTrackbarPos("SAT Max", "HSV")
        v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
        v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

        lower = np.array([h_min,s_min,v_min])
        upper = np.array([h_max,s_max,v_max])
        mask = cv2.inRange(imgHsv,lower,upper)
        result = cv2.bitwise_and(ImageSlave,ImageSlave, mask = mask)
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
        imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
        threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
        threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
        imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
        kernel = np.ones((5, 5))
        imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
        getContours(imgDil, imgContour)
        display(imgContour)

        # Flight the drone for first time usage
        if startCounter == 0:
            sleep(0.1)
            slave.takeoff()
            startCounter = 1

        # print("Now direction is: ", direction)

        if direction == 3:
           slave.up_down_velocity = 15
           #print(dir)
        elif direction == 4:
           slave.up_down_velocity = -15
           #print(dir)
        elif direction == 1:
           slave.yaw_velocity = -25
           #print(dir)
        elif direction == 2:
           slave.yaw_velocity = 25
           #print(dir)
        elif direction == 99:
           slave.for_back_velocity = 15
           #print(dir)
        elif direction == -99:
           slave.for_back_velocity = -15
           #print(dir)


        else:
           slave.left_right_velocity = 0; slave.for_back_velocity = 0; slave.up_down_velocity = 0; slave.yaw_velocity = 0

       # Received the command from image detection
        if slave.send_rc_control:
           slave.send_rc_control(slave.left_right_velocity, slave.for_back_velocity, slave.up_down_velocity, slave.yaw_velocity)


        stack = stackImages(0.9, ([ImageSlave, result], [imgDil, imgContour]))
        cv2.imshow('Horizontal Stacking', stack)

        # Key input 'Q' as landing button
        if cv2.waitKey(1) & 0xFF == ord('q'):
            slave.land()
            break


    # Destroy (Close) all windows after drone landing
    # cap2.release()
    cv2.destroyAllWindows()