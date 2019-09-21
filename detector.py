import numpy as np
import cv2
import time

# open the camera
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# Set the initial time
aeg = time.time()

# set the kernel size for blurring
kernel = 9

# set the kernel size for morphology, the first is a matrix and the second is an integer
morph = np.ones((5, 5), np.uint8)
morphvalue = 5

# global values for both the ball's and basket's x position
ball_x = 0
basket_x = 0

# Read global variables for detecting the ball.
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
    file.close()
except:
    lH = 0
    lS = 193
    lV = 138
    hH = 14
    hS = 255
    hV = 255

# Read global variables for detecting the basket.
try:
    file = open("thresh_basket.txt")
    f = list(file)
    f = [x.strip() for x in f]
    lH_ = int(f[0])
    lS_ = int(f[1])
    lV_ = int(f[2])
    hH_ = int(f[3])
    hS_ = int(f[4])
    hV_ = int(f[5])
    file.close()

except:
    lH_ = 0
    lS_ = 193
    lV_ = 138
    hH_ = 14
    hS_ = 255
    hV_ = 255


def thresholding(frame, lowerLimits, upperLimits):
    # RGB to HSV colour space
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # BLUR averaging
    frame = cv2.blur(frame, (kernel, kernel))

    # Operations on the frame
    thresholded = cv2.inRange(frame, lowerLimits, upperLimits)

    thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, morph)

    return thresholded


def blob_detector(frame, thresholded, detector):
    # BLOB DETECTION
    keypoints = detector.detect(thresholded)
    frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    x = 10000

    if len(keypoints) > 0:
        keypoint_largest = keypoints[0]
        current_max_size = 0
        for keypoint in keypoints:
            if keypoint.size > current_max_size:
                current_max_size = keypoint.size
                keypoint_largest = keypoint

        x = int(keypoint_largest.pt[0])
        y = int(keypoint_largest.pt[1])
        tekst = "x: " + str(x) + " y: " + str(y)
        cv2.putText(frame, tekst, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # for keypoint in keypoints:
    #     x = int(keypoint.pt[0])
    #     y = int(keypoint.pt[1])
    #     tekst = "x: " + str(x) + " y: " + str(y)
    #     cv2.putText(frame, tekst, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return frame, x

def draw_centerline_on_frame(frame, cap) :
    # x1 = frame center; y1: frame height (top); x2: frame center; y2: frame height (bottom).
    x1 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2)
    y1 = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    x2 = x1
    y2 = 0
    # Set line attributes.
    line_thickness = 1
    line_color = (255, 0, 0)
    cv2.line(frame, (x1, y1), (x2, y2), line_color, line_thickness)

    return frame


def find_contours(frame, thresholded):
    contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cx = 10000
    sorted_contours = sorted(contours, key=cv2.contourArea)

    if len(sorted_contours) > 0:
        cv2.drawContours(frame, sorted_contours[-1], -1, (0, 255, 0), 3)

    try:
        if len(sorted_contours) > 0:
            # image moment
            m = cv2.moments(sorted_contours[-1])
            # print(m.keys())

            # The centroid point
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])
            # print(cx)

            # for contour in contours:
            #     cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)
    except:
        cx = 10000

    return frame, cx


# Detector configuration
blobparams = cv2.SimpleBlobDetector_Params()
blobparams.minArea = 100
blobparams.maxArea = 1000000
blobparams.filterByColor = True
blobparams.blobColor = 255
detector = cv2.SimpleBlobDetector_create(blobparams)


# # read one frame to detect image size
# ret, frame = cap.read()
# # print image center
# width = len(frame[0])
# img_center = width / 2

def main(q_ball, q_basket, q_stop):
    global ball_x
    global basket_x

    while True:

        # Check for stop signals
        if not q_stop.empty():
            cap.release()
            cv2.destroyAllWindows()
            # When everything done, release the capture
            print('closing detector')
            return

        # Write the framerate
        # eelmine_aeg = aeg
        # aeg = time.time()
        # framerate = 1 / (aeg - eelmine_aeg)
        # cv2.putText(frame, str(framerate), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # read the image from the camera
        ret, frame = cap.read()

        # Detect both the ball and the basket in a loop.

        # Set the thresholding parameters.
        lowerLimits_ball = np.array([lH, lS, lV])
        upperLimits_ball = np.array([hH, hS, hV])
        lowerLimits_basket = np.array([lH_, lS_, lV_])
        upperLimits_basket = np.array([hH_, hS_, hV_])

        # outimage = cv2.bitwise_and(frame, frame, mask=thresholded)

        # Operations concerning the ball.
        thresholded_ball = thresholding(frame, lowerLimits_ball, upperLimits_ball)
        # frame = blob_detector(frame, thresholded_ball, detector)
        frame, ball_x = blob_detector(frame, thresholded_ball, detector)

        q_ball.put(ball_x)
        # Operations concerning the basket.
        thresholded_basket = thresholding(frame, lowerLimits_basket, upperLimits_basket)
        # frame = blob_detector(frame, thresholded_basket, detector)
        frame, basket_x = find_contours(frame, thresholded_basket)
        frame = draw_centerline_on_frame(frame, cap)

        cv2.imshow('Frame', frame)
        cv2.imshow('Thresh Ball', thresholded_ball)
        cv2.imshow('Thresh Basket', thresholded_basket)

        # Quit the program when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            q_stop.put(True)
            cap.release()
            cv2.destroyAllWindows()
            # When everything done, release the capture
            print('closing detector')
            return


#main()