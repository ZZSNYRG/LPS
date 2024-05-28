import cv2 
import numpy as np
import serial
import struct
from math import sqrt, atan2, degrees
import time
import multiprocessing

cmLength = multiprocessing.Value("i", 250)
cmWidth = multiprocessing.Value("i", 200)
camNum = multiprocessing.Value("i", 0)
portNum = multiprocessing.Value("i", 1)

#resolution = [1920, 1080]
resolution = [960, 720]
coordsQueue = multiprocessing.Queue(maxsize=1)

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

def warpPerspective(frame, srcPts, dstSize):
    dstPts = np.array([[0, 0], [dstSize[0] - 1, 0], [dstSize[0] - 1, dstSize[1] - 1], [0, dstSize[1] - 1]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(srcPts, dstPts)
    warpedFrame = cv2.warpPerspective(frame, matrix, dstSize)
    return warpedFrame

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

def dist(xy2 : int, xy1 : int):
    distance =  sqrt((xy2[0] - xy1[0])**2 + (xy2[1] - xy1[1])**2)
    return distance

def getCoords(frame, coords, cmLength, cmWidth):
    # Triangle not detected or higher order shape
    if coords is None or len(coords) != 2:
        coordinates = []
        return
    else:
        x1, y1 = coords[1]
        x2, y2 = coords[0]
        cv2.line(frame, (x1, y1), (x2, y2), (0,0,255), 2, -1)
        cv2.circle(frame, (x1, y1), 8, (0,0,255), -1) # Postioning system center
        y1 = frame.shape[0] - y1
        y2 = frame.shape[0] - y2
        angleRadians = atan2((y2 - y1), (x2 - x1)) 
        angleDegrees = (int(degrees(angleRadians)) + 360) % 360
        #print(f"{x1}, {y1}, {angleDegrees}")
        xCoord = round((x1 * (cmLength.value/frame.shape[1])),None)
        yCoord = round((y1 * (cmWidth.value/frame.shape[0])),None) 
        coordinates = [xCoord, yCoord, angleDegrees]
    return coordinates

def displayData(coordinates, frame):
    textbox = cv2.rectangle(frame, (100, 100 -25), (100 + 250, 100 + 10), (255,255,255), -1)
    if coordinates is None or coordinates == []:
        label = " Not found "
    else:
        label = " X: " + str(coordinates[0]) + "cm," + " Y: " + str(coordinates[1]) + "cm," + " A: " + str(coordinates[2]) + "deg"
    cv2.putText(frame, label, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)    
    return

def dataTxRx(coordsQueue, portNum):
    global errorMsg
    errorMsg = 0xFFFFFFFFFFFFFFFF
    port = f"COM{portNum.value}" 
    baudrate = 115200
    serialConnectFlag = 0
    
    while True:
        objctCoords = coordsQueue.get()
        if serialConnectFlag == 0:
            try:
                ser = serial.Serial(port, baudrate)
                print("Connection established")
                serialConnectFlag = 1
                time.sleep(1)
            except: 
                print("Connection to port failed")
                serialConnectFlag = 0
                time.sleep(1)
                continue
            continue

        try:
            sendData(objctCoords, ser)
        except:
            print("Data send error")
            serialConnectFlag = 0
            time.sleep(1)

        try:
            rcvData(ser)
        except:
            print("Data receive error")
            serialConnectFlag = 0
            time.sleep(1)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            ser.close()
            break
    return

def sendData(objctCoords, serialHandle):
    global errorMsg
    if objctCoords is None or objctCoords == (0, 0, 0) or objctCoords == ():
        packed_data = struct.pack('>Q', errorMsg)
    else:
        packed_data = struct.pack('>Q', (objctCoords[0] << 32) | (objctCoords[1] << 16) | (objctCoords[2]))
    serialHandle.write(packed_data)
    return

def rcvData(serialHandle):
    data = serialHandle.readline()
    print(data)
    return

def processingImg(coordsQueue, cmLength, cmWidth, camNum):
    cap = cv2.VideoCapture(camNum.value, cv2.CAP_DSHOW)
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

        # Only run corner select task if there has been a callback to the selection function
        if selectCornerFlag == 1:

            # Sorts coordinates into arbitrary established order
            if len(cornerCoords) >= 4:
                if cornerCoords[0][1] < cornerCoords[1][1]: # Possible error if values are equal
                    pt1 = [cornerCoords[0][0], cornerCoords[0][1]]
                    pt4 = [cornerCoords[1][0], cornerCoords[1][1]]
                else: 
                    pt4 = [cornerCoords[0][0], cornerCoords[0][1]]
                    pt1 = [cornerCoords[1][0], cornerCoords[1][1]]

                if cornerCoords[3][1] > cornerCoords[2][1]: # Possible error if values are equal
                    pt3 = [cornerCoords[3][0], cornerCoords[3][1]]
                    pt2 = [cornerCoords[2][0], cornerCoords[2][1]]
                else: 
                    pt2 = [cornerCoords[3][0], cornerCoords[3][1]]
                    pt3 = [cornerCoords[2][0], cornerCoords[2][1]]
                srcPts = np.array([pt1, pt2, pt3, pt4], dtype=np.float32)
                selectCornerFlag = 0

        # Refresh warped frame       
        if srcPts is None:
            warpedFrame=filteredFrame
        else:
            frameWidth = srcPts[1][0] - srcPts[0][0]
            frameHeight = srcPts[3][1] - srcPts[0][1]
            scaleFactor = 0.08
            while frameHeight <= resolution[1] or frameWidth <= resolution[0]:
                frameWidth += (scaleFactor*frameWidth)
                frameHeight += (scaleFactor*frameHeight)
                if frameHeight > resolution[1] or frameWidth > resolution[0]:
                    frameWidth -= (scaleFactor*frameWidth)
                    frameHeight -= (scaleFactor*frameHeight)
                    break
            frameWidth = int(frameWidth)
            frameHeight = int(frameHeight)
            warpedFrame = warpPerspective(filteredFrame, srcPts, (frameWidth, frameHeight))

        cv2.imshow("Original", drawnFrame)
        cv2.imshow("Warped", warpedFrame)

        imgHSV = cv2.cvtColor(warpedFrame, cv2.COLOR_BGR2HSV)
        cv2.namedWindow("Warped")
        cv2.setMouseCallback("Warped", onMouse, imgHSV)
        # No object color has been detected, return to fetch new frame
        if objctHSV[0] == False:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        lowerObjctHSV, upperObjctHSV = rangingHSV(objctHSV, 30, 70, 70)
        objctContours = getContours(imgHSV, lowerObjctHSV, upperObjctHSV, 2)
        objctCoords = getCentroids(drawnFrame, objctContours, False) 
        genCoords = getCoords(warpedFrame, objctCoords, cmLength, cmWidth)
        displayData(genCoords, warpedFrame)

        if coordsQueue.full() == True:
            queueTrash = coordsQueue.get()
        coordsQueue.put(genCoords)

        cv2.imshow("Original", drawnFrame)
        cv2.imshow("Warped", warpedFrame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return


def main():
    multiprocessing.freeze_support()
    
    try:
        cmLength.value = int(input("Enter length of area in centimeters (X axis): "))
    except:
        print("Default value assigned of 300")
        cmLength.value = 300

    try:
        cmWidth.value = int(input("Enter width of area in centimeters (Y axis): "))
    except:
        print("Default value assigned of 200")
        cmWidth.value = 200

    try:
        camNum.value = int(input("Enter camera number (0,1,2...): "))
    except:
        print("Default value assigned of 0")
        camNum.value = 0

    try:
        portNum.value = int(input("Enter com port number (COMX) of RF sender: "))
    except:
        print("Default value assigned of 8")
        portNum.value = 8

    p1 = multiprocessing.Process(target=processingImg, args=(coordsQueue, cmLength, cmWidth, camNum,))
    p2 = multiprocessing.Process(target=dataTxRx, args=(coordsQueue, portNum,))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

if __name__ == "__main__":
    main()