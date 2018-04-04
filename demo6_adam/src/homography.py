#!/usr/bin/env python

import numpy as np
import cv2
from matplotlib import pyplot as plt

def draw_matches(img1, kp1, img2, kp2, matches, color=None):
    """Draws lines between matching keypoints of two images.
    Keypoints not in a matching pair are not drawn.
    Places the images side by side in a new image and draws circles
    around each keypoint, with line segments connecting matching pairs.
    You can tweak the r, thickness, and figsize values as needed.
    Args:
        img1: An openCV image ndarray in a grayscale or color format.
        kp1: A list of cv2.KeyPoint objects for img1.
        img2: An openCV image ndarray of the same format and with the same
        element type as img1.
        kp2: A list of cv2.KeyPoint objects for img2.
        matches: A list of DMatch objects whose trainIdx attribute refers to
        img1 keypoints and whose queryIdx attribute refers to img2 keypoints.
        color: The color of the circles and connecting lines drawn on the images.
        A 3-tuple for color images, a scalar for grayscale images.  If None, these
        values are randomly generated.
    """
    # We're drawing them side by side.  Get dimensions accordingly.
    # Handle both color and grayscale images.
    if len(img1.shape) == 3:
        new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1], img1.shape[2])
    elif len(img1.shape) == 2:
        new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1])
        new_img = np.zeros(new_shape, type(img1.flat[0]))
        # Place images onto the new image.
        new_img[0:img1.shape[0],0:img1.shape[1]] = img1
        new_img[0:img2.shape[0],img1.shape[1]:img1.shape[1]+img2.shape[1]] = img2

        # Draw lines between matches.  Make sure to offset kp coords in second image appropriately.
        r = 15
        thickness = 2
        if color:
            c = color
            for m in matches:
                # Generate random color for RGB/BGR and grayscale images as needed.
                if not color:
                    c = np.random.randint(0,256,3) if len(img1.shape) == 3 else np.random.randint(0,256)
                    # So the keypoint locs are stored as a tuple of floats.  cv2.line(), like most other things,
                    # wants locs as a tuple of ints.
                    end1 = tuple(np.round(kp1[m.trainIdx].pt).astype(int))
                    end2 = tuple(np.round(kp2[m.queryIdx].pt).astype(int) + np.array([img1.shape[1], 0]))
                    cv2.line(new_img, end1, end2, c, thickness)
                    cv2.circle(new_img, end1, r, c, thickness)
                    cv2.circle(new_img, end2, r, c, thickness)

                    plt.figure(figsize=(15,15))
                    plt.imshow(new_img)
                    plt.show()

MIN_MATCH_COUNT = 10

img1 = cv2.imread('ualberta_logo.png',0)          # queryImage
img2 = cv2.imread('test_image.png',0) # trainImage

#cv2.imshow("img1",img1)
#cv2.waitKey(200)
#cv2.imshow("img2",img2)
#cv2.waitKey(200)

# Initiate orb detector
orb = cv2.ORB()

# find the keypoints and descriptors with orb
kp1 = orb.detect(img1,None)
kp1, des1 = orb.compute(img1, kp1)
kp2 = orb.detect(img2,None)
kp2, des2 = orb.compute(img2, kp2)

FLANN_INDEX_LSH = 6
index_params= dict(algorithm = FLANN_INDEX_LSH,
           table_number = 6, # 12
           key_size = 12,     # 20
           multi_probe_level = 1) #2
search_params = dict(checks = 50)

flann = cv2.FlannBasedMatcher(index_params, search_params)

matches = flann.knnMatch(des1, des2, k=2)

# Need to draw only good matches, so create a mask
matchesMask = [[0,0] for i in xrange(len(matches))]

# ratio test as per Lowe's paper
#  for m,n in matches:
#      if m.distance < 0.7*n.distance:
#          matchesMask[i]=[1,0]

draw_params = dict(color = (0,255,0)
                   )

draw_matches(img1,kp1,img2,kp2,matches,**draw_params)

#plt.imshow(img3,),plt.show()

while True:
    pass
