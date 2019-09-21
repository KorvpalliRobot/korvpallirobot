import numpy as np
import cv2
import time

# open the camera
cap = cv2.VideoCapture(1)

# Set the initial time
aeg = time.time()

# set the kernel size for blurring
kernel = 5

# set the kernel size for morphology, the first is a matrix and the second is an integer
morph = np.ones((5, 5), np.uint8)
morphvalue = 5

# Selector to choose whether to update the threshold values for the ball or the basket.
selector = 0

# Read global variables for trackbars from thresh.txt
try:
    file = open("thresh_ball.txt")
    f = list(file)
    f = [x.strip() for x in f]
    lH = int(f[0])
    lS = int(f[1])
    lV = int(f[2])
    hH = int(f[3])
    hS = int(f[4])
    hV = int(f[5])

except:
    lH = 0
    lS = 193
    lV = 138
    hH = 14
    hS = 255
    hV = 255

file.close()

""" igaks juhuks vanad väärtused:
    lH = 0
    lS = 242
    lV = 131
    hH = 34
    hS = 255
    hV = 255
"""


# A callback function for a trackbar
# It is triggered every time the slider on trackbar is used
def updatelH(new_value):
    # make sure to write the new value into the global variable
    global lH
    lH = new_value
    return


def updatelS(new_value):
    # make sure to write the new value into the global variable
    global lS
    lS = new_value
    return


def updatelV(new_value):
    # make sure to write the new value into the global variable
    global lV
    lV = new_value
    return


def updatehH(new_value):
    # make sure to write the new value into the global variable
    global hH
    hH = new_value
    return


def updatehS(new_value):
    # make sure to write the new value into the global variable
    global hS
    hS = new_value
    return


def updatehV(new_value):
    # make sure to write the new value into the global variable
    global hV
    hV = new_value
    return


def updatekernel(new_value):
    # make sure to write the new value into the global variable
    if new_value % 2 != 0:
        global kernel
        kernel = new_value
    else:
        kernel = new_value + 1
    return


# updatemorph updates the both integer "morphvalue" for trackbars and the matrix for morphology
def updatemorph(new_value):
    # make sure to write the new value into the global variable
    if new_value % 2 != 0:
        global morph
        morph = np.ones((new_value, new_value), np.uint8)
        global morphvalue
        morphvalue = new_value
    else:
        morph = np.ones((new_value + 1, new_value + 1), np.uint8)
        morphvalue = new_value + 1
    return


# Selector to choose whether to update the threshold values for the ball or the basket.
# 0 == ball; 1 == basket.
# This is a function to update the corresponding trackbar.
def update_selector(new_value):
    # make sure to write the new value into the global variable
    global selector
    selector = new_value
    return


# Create a window
cv2.namedWindow("Trackbars")
# Attach a trackbar to a window
cv2.createTrackbar("Ball == 0; basket == 1", "Trackbars", selector, 1, update_selector)
cv2.createTrackbar("lH", "Trackbars", lH, 255, updatelH)
cv2.createTrackbar("lS", "Trackbars", lS, 255, updatelS)
cv2.createTrackbar("lV", "Trackbars", lV, 255, updatelV)
cv2.createTrackbar("hH", "Trackbars", hH, 255, updatehH)
cv2.createTrackbar("hS", "Trackbars", hS, 255, updatehS)
cv2.createTrackbar("hV", "Trackbars", hV, 255, updatehV)

# Trackbar for blur kernel size
cv2.createTrackbar("Blur kernel size", "Trackbars", kernel, 19, updatekernel)
cv2.createTrackbar("Morph kernel size", "Trackbars", morphvalue, 19, updatemorph)

# Detector configuration
blobparams = cv2.SimpleBlobDetector_Params()
blobparams.minArea = 10
blobparams.maxArea = 1000000
blobparams.filterByColor = True
blobparams.blobColor = 255
detector = cv2.SimpleBlobDetector_create(blobparams)

while True:
    # read the image from the camera
    ret, frame = cap.read()

    # RGB to HSV colour space
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # BLUR averaging
    frame = cv2.blur(frame, (kernel, kernel))

    # BLUR Bilateral
    # frame = cv2.bilateralFilter(frame,kernel,75,75)

    # BLUR Gaussian
    # frame = cv2.GaussianBlur(frame, (kernel, kernel), 0)

    lowerLimits = np.array([lH, lS, lV])
    upperLimits = np.array([hH, hS, hV])

    # Our operations on the frame come here
    thresholded = cv2.inRange(frame, lowerLimits, upperLimits)
    # thresholded = cv2.bitwise_not(thresholded)

    # Morphology
    # thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, morph)
    thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, morph)
    # thresholded = cv2.erode(thresholded,morph,iterations = 1)

    #outimage = cv2.bitwise_and(frame, frame, mask=thresholded)

    # Write the framerate
    eelmine_aeg = aeg
    aeg = time.time()
    framerate = 1 / (aeg - eelmine_aeg)
    cv2.putText(frame, str(framerate), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # BLOB DETECTION
    keypoints = detector.detect(thresholded)
    frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    for keypoint in keypoints:
        x = int(keypoint.pt[0])
        y = int(keypoint.pt[1])
        tekst = "x: " + str(x) + " y: " + str(y)
        cv2.putText(frame, tekst, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Original', frame)
    cv2.imshow('Thresh', thresholded)

    # Display the resulting frame
    #cv2.imshow('Processed', outimage)

    # Quit the program when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
print('closing program')

# Writing threshold variables to file
# Again, variable "selector" chooses which file to write.
if selector == 0:
    f = open("thresh_ball.txt", "w")
elif selector == 1:
    f = open("thresh_basket.txt", "w")
else:
    f = open("thresh.txt", "w")

f.write(str(lH) + "\n")
f.write(str(lS) + "\n")
f.write(str(lV) + "\n")
f.write(str(hH) + "\n")
f.write(str(hS) + "\n")
f.write(str(hV) + "\n")
f.close()

cap.release()
cv2.destroyAllWindows()
