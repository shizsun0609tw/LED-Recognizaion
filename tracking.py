import cv2
import numpy as np
import time
import copy
import csv

LOWER = {'red':np.array([0, 180, 180]), 'green':np.array([30, 180, 180]), 'blue':np.array([90,  180, 180])}
UPPER = {'red':np.array([25, 255, 255]), 'green':np.array([80, 255, 255]), 'blue':np.array([130, 255, 255])}
COLOR = {'red':(0, 0, 255), 'green':(0, 255, 0), 'blue':(255, 0, 0)}
init_S_Lower = 180
init_S_Upper = 255
init_V_Lower = 180
init_V_Upper = 255 
NOISE_RADIUS = 10
GROUP_RADIUS = 100
TIME_ERROR = 0.1
FPS = 20.0
inWidth = 640
inHeight = 480

class LED(object):
    def __init__(self, light, center, time):
        self.light = light
        self.center = center
        self.time_point = [time]
        self.HZ = [0]

def modifyParameter(x):
    global NOISE_RADIUS, GROUP_RADIUS
    
    UPPER['red'][0] = cv2.getTrackbarPos('R_upper', 'tracking')
    LOWER['red'][0] = cv2.getTrackbarPos('R_lower', 'tracking')
    UPPER['red'][1] = cv2.getTrackbarPos('S_upper', 'tracking')
    LOWER['red'][1] = cv2.getTrackbarPos('S_lower', 'tracking')
    UPPER['red'][2] = cv2.getTrackbarPos('V_upper', 'tracking')
    LOWER['red'][2] = cv2.getTrackbarPos('V_lower', 'tracking')
    UPPER['green'][0] = cv2.getTrackbarPos('G_upper', 'tracking')
    LOWER['green'][0] = cv2.getTrackbarPos('G_lower', 'tracking')
    UPPER['green'][1] = cv2.getTrackbarPos('S_upper', 'tracking')
    LOWER['green'][1] = cv2.getTrackbarPos('S_lower', 'tracking')
    UPPER['green'][2] = cv2.getTrackbarPos('V_upper', 'tracking')
    LOWER['green'][2] = cv2.getTrackbarPos('V_lower', 'tracking')
    UPPER['blue'][0] = cv2.getTrackbarPos('B_upper', 'tracking')
    LOWER['blue'][0] = cv2.getTrackbarPos('B_lower', 'tracking')
    UPPER['blue'][1] = cv2.getTrackbarPos('S_upper', 'tracking')
    LOWER['blue'][1] = cv2.getTrackbarPos('S_lower', 'tracking')
    UPPER['blue'][2] = cv2.getTrackbarPos('V_upper', 'tracking')
    LOWER['blue'][2] = cv2.getTrackbarPos('V_lower', 'tracking')
    NOISE_RADIUS = cv2.getTrackbarPos('NOISE_RADIUS', 'tracking')
    GROUP_RADIUS = cv2.getTrackbarPos('GROUP_RADIUS', 'tracking')

def setTrackbar():
    init_UPPER = copy.deepcopy(UPPER)
    init_LOWER = copy.deepcopy(LOWER)
    init_NOISE_RADIUS = NOISE_RADIUS
    init_GROUP_RADIUS = GROUP_RADIUS

    cv2.namedWindow('tracking', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('tracking', 800, 800)
    cv2.createTrackbar('R_lower', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('R_upper', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('G_lower', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('G_upper', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('B_lower', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('B_upper', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('S_lower', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('S_upper', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('V_lower', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('V_upper', 'tracking', 0, 255, modifyParameter)
    cv2.createTrackbar('GROUP_RADIUS', 'tracking', 0, 500, modifyParameter)
    cv2.createTrackbar('NOISE_RADIUS', 'tracking', 0, 50, modifyParameter)

    cv2.setTrackbarPos('R_lower', 'tracking', init_LOWER['red'][0])
    cv2.setTrackbarPos('R_upper', 'tracking', init_UPPER['red'][0])
    cv2.setTrackbarPos('G_lower', 'tracking', init_LOWER['green'][0])
    cv2.setTrackbarPos('G_upper', 'tracking', init_UPPER['green'][0])
    cv2.setTrackbarPos('B_lower', 'tracking', init_LOWER['blue'][0])
    cv2.setTrackbarPos('B_upper', 'tracking', init_UPPER['blue'][0])
    cv2.setTrackbarPos('S_lower', 'tracking', init_S_Lower)
    cv2.setTrackbarPos('S_upper', 'tracking', init_S_Upper)
    cv2.setTrackbarPos('V_lower', 'tracking', init_V_Lower)
    cv2.setTrackbarPos('V_upper', 'tracking', init_V_Upper)
    cv2.setTrackbarPos('GROUP_RADIUS', 'tracking', init_GROUP_RADIUS)
    cv2.setTrackbarPos('NOISE_RADIUS', 'tracking', init_NOISE_RADIUS)

def selectColor(img, color):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = np.ones((5, 5), np.uint8)
    thresh = cv2.inRange(img, LOWER[color], UPPER[color])
    thresh = cv2.erode(thresh, mask)
    thresh = cv2.dilate(thresh, mask)
    thresh = cv2.dilate(thresh, mask)
    thresh = cv2.dilate(thresh, mask)
    thresh = cv2.GaussianBlur(thresh, (5, 5), 0)
    thresh = cv2.medianBlur(thresh, 5)

    return thresh

def contour2Group(contour):
    center, radius = cv2.minEnclosingCircle(contour)

    return center, radius

def calCenter(group):
    countX = []
    countY = []
    for key, value in group.items():
        countX.append(value.center[0])
        countY.append(value.center[1])

    return (int(sum(countX) / len(countX)), int(sum(countY) / len(countY)))

def contourInGroup(center, group):
    if cv2.norm(center, calCenter(group)) < GROUP_RADIUS:
        return True
    else:
        return False

def updateColorInGroup(center, color, group):
    if color in group:
        group[color].center = center
        if group[color].light == False and now != group[color].time_point[-1]:
            group[color].light = True
            group[color].HZ.append(1 / (now - group[color].time_point[-1]))
            group[color].time_point.append(now)
    else:
        group[color] = LED(True, center, now)
    return group

def matchGroup(center, color, groups):
    groupExist = False
    
    for group in groups:
        if contourInGroup(center, group):
            groupExist = True
            group = updateColorInGroup(center, color, group)
            break

    if groupExist == False:
        groups.append({color:LED(True, center, now)})

def updateGroups(frame, groups):
    for color, value in UPPER.items():
        img = selectColor(frame.copy(), color)
        contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            center, radius = contour2Group(contour)
            if radius > NOISE_RADIUS:
                matchGroup(center, color, groups)
                cv2.circle(frame, (int(center[0]), int(center[1])), int(radius), COLOR[color], 3)
                cv2.rectangle(frame, \
                    (int(center[0] - radius), int(center[1] - radius)), \
                    (int(center[0] + radius), int(center[1] + radius)), \
                    (255, 255, 255), 5)

    return groups

def isGroupLEDLightOn(group):
    for color, LED in group.items():
        if LED.light == True:
            return True
    return False

def paintGroups(frame, groups):
    for group in groups:
        if isGroupLEDLightOn(group) == True: 
            center = calCenter(group)
            leftUpCoordinate = (center[0] - GROUP_RADIUS//2, center[1] - GROUP_RADIUS//2)
            rightDownCoordinate = (center[0] + GROUP_RADIUS//2, center[1] + GROUP_RADIUS//2)

            cv2.rectangle(frame, leftUpCoordinate, rightDownCoordinate, (0, 255, 0), 5)
            offset = 50
            for color, LED in group.items():
                textPos = (leftUpCoordinate[0] - 10, leftUpCoordinate[1] - offset)
                cv2.putText(frame, color + ":" + str(round(group[color].HZ[-1], 3)), textPos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR[color], 2)
                offset += 20

    return frame

def resetGroups(groups):
    for group in groups:
        for color, value in group.items(): 
            if (group[color].light == True and now - group[color].time_point[-1] > TIME_ERROR): 
                group[color].light = False
            
    return groups

def writeFile(groups):
    with open('output.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        for i in range(len(groups)):
            writer.writerow(['Group ' + str(i)])
            writer.writerow(['Red', ' ', 'Green', ' ', 'Blue'])
            writer.writerow(['HZ', 'Time', 'HZ', 'Time', 'HZ', 'Time'])
            count = 0
            row = []
            color = ['red', 'green', 'blue']
            while ('red' in groups[i] and count < len(groups[i]['red'].HZ)) \
                or ('green' in groups[i] and count < len(groups[i]['green'].HZ)) \
                or ('blue' in groups[i] and count < len(groups[i]['blue'].HZ)):
                for c in range(len(color)):
                    if color[c] in groups[i] and count < len(groups[i][color[c]].HZ):
                        row.append(groups[i][color[c]].HZ[count])
                        row.append(groups[i][color[c]].time_point[count] - start_time_point)
                    else:
                        row.append(" ")
                        row.append(" ")
                
                writer.writerow(row)
                row = []
                count += 1
            writer.writerow(' ')

def writeVideo(frame):
    output_video.write(frame)

#--------------------main--------------------# 
setTrackbar()
groups = []
cap = cv2.VideoCapture(0)
start_time_point = time.time()
now = time.time()

fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
output_video = cv2.VideoWriter('output.mp4', fourcc, FPS, (inWidth, inHeight))

while True:
    ret, frame = cap.read()
    if ret == False:
        break

    #  Update  #
    now = time.time()
    groups = updateGroups(frame, groups)
    frame = paintGroups(frame, groups)
    groups = resetGroups(groups)
    writeVideo(frame)

    cv2.imshow('tracking', frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q') or cv2.getWindowProperty('tracking', cv2.WND_PROP_VISIBLE) < 1:
        writeFile(groups)
        break

output_video.release()
cap.release()
cv2.destroyAllWindows()