import cv2 
import numpy as np
import serial
import struct
from math import sqrt, atan2, degrees
import math
import time
import multiprocessing
import traceback

#resolution = [1920, 1080]
resolution = [960, 720]

def onMouse(event, x, y, flags, frame):
    global selectCornerFlag
    if event == cv2.EVENT_LBUTTONDOWN:
        if flags & cv2.EVENT_FLAG_CTRLKEY:
            global cornerHSV
            cornerHSV = frame[y, x]
            selectCornerFlag = 1
            #print("HSV color = {}".format(cornerHSV))
        else:
            global objctHSV
            objctHSV = frame[y, x]
            #print("HSV color = {}".format(objctHSV))

def rangingHSV(refrHSV, toleranceHue: int, toleranceSaturation: int, toleranceVal: int):
    lowerHSV = np.array([refrHSV[0]-toleranceHue, refrHSV[1]-toleranceSaturation, refrHSV[2]-toleranceVal])
    upperHSV = np.array([refrHSV[0]+toleranceHue, refrHSV[1]+toleranceSaturation, refrHSV[2]+toleranceVal])
    return lowerHSV, upperHSV

def getContours(imgHSV, lowerBound, upperBound, contourNum: int):
    mask = cv2.inRange(imgHSV, lowerBound, upperBound)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=2)
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    sortedContours = sorted(contours, key=lambda c: cv2.contourArea(c), reverse=True)
    topContours = sortedContours[:contourNum]
    return topContours

def getCentroids(frame, contours, sortFlag : bool):
    coords = []
    i = 0
    for contour in contours:
        M = cv2.moments(contour)
        # Calculate the centroid coordinates
        if M["m00"] != 0:  # Avoid division by zero
            coords.append((int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
            #cv2.circle(frame, coords[i], 5, 255, -1)
            i+=1
    if sortFlag == True:     
        coordinates = sorted(coords, key=lambda x: x[0]) 
    else: 
        coordinates = coords
    return coordinates

def processingImg():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    global cornerHSV, objctHSV, selectCornerFlag
    srcPts = None
    cornerHSV = [False, 255, 255]
    objctHSV = [False, 255, 255]
    objctCoords = []

    while True:

        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        filteredFrame = cv2.convertScaleAbs(frame, alpha=0.9, beta=0)
        #filteredFrame = cv2.bilateralFilter(brighterFrame, 9, 75, 75)
        drawnFrame=filteredFrame.copy() # Draw circles and contours in a new frame to maintain original unaltered 
        
        # Define the four source points (ROI corners) for the perspective transformation
        imgHSV = cv2.cvtColor(filteredFrame, cv2.COLOR_BGR2HSV)
        cv2.imshow("Original", drawnFrame)
        cv2.setMouseCallback("Original", onMouse, imgHSV)
        # No corner color has been detected, return to fetch new frame
        if cornerHSV[0] == False:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        lowerCornerHSV, upperCornerHSV = rangingHSV(cornerHSV, 15, 40, 40)
        cornersContours = getContours(imgHSV, lowerCornerHSV, upperCornerHSV, 4)
        cv2.drawContours(drawnFrame, cornersContours, -1, (0, 255, 0), 2)
        cornerCoords = getCentroids(drawnFrame, cornersContours, True)
        cv2.imshow("Original", drawnFrame)

        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return

def dataTxRx():
    global msNowConnect, errorMsg
    errorMsg = 0xFFFF0FFF0FFF0FFF
    port = "COM8" 
    baudrate = 115200
    msNowConnect = 0
    serialConnectFlag = 0
    while True:
        msLast = msNowConnect
        msNowConnect = time.time() % 1
        if serialConnectFlag == 0:
            if msNowConnect < msLast: 
                try:
                    ser = serial.Serial(port, baudrate)
                    print("Connection established")
                    serialConnectFlag = 1
                except: 
                    print("Connection to port failed")
                    serialConnectFlag = 0
                    continue
                continue
            continue
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    return

def main():

    try: 
        p1 = multiprocessing.Process(target=processingImg,)
        p2 = multiprocessing.Process(target=dataTxRx,)

        p1.start()
        p2.start()

        p1.join()
        p2.join()
    except:
        traceback.print_exc()
        time.sleep(10)

if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()