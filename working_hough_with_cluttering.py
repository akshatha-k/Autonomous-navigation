import numpy as np
import cv2
import sys
import math
import statistics

def main():
    src=cv2.imread('./imagesRGB/RGB01.png')
    gray= cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gray',gray)
    gray=cv2.bitwise_not(gray)
    gray=cv2.Sobel(gray, cv2.CV_8U, 0, 1, ksize=5)
    #cv2.imshow("biwise",gray)
    bw=cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, -2)
    # cv2.imshow('binary', bw)
    horizontal=np.copy(bw)
    cols =rizontal.shape[1]
    horizontal_size = cols // 30
    # Create structure element for extracting horizontal lines through morphology operations
    horizontalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (horizontal_size, 1))
    # Apply morphology operations
    horizontal = cv2.erode(horizontal, horizontalStructure)
    horizontal = cv2.dilate(horizontal, horizontalStructure)
    kernel = np.ones((5,5),np.uint8)
    horizontal= cv2.dilate(horizontal,kernel,iterations=1)
    horizontal= cv2.erode(horizontal,horizontalStructure,iterations=2)
    # Show extracted horizontal lines
    # cv2.imshow('horizontal',horizontal)
    lines=cv2.HoughLines(horizontal,1,math.pi/180,200)

    for value in lines:
        rho = value[0][0]
        theta =value[0][1]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        cv2.line(src,(x1,y1),(x2,y2),(0,0,255),2)
    cv2.imshow('Hough',src)

    final=cv2.bitwise_and(src,src, mask=horizontal)
    cv2.imshow('final',final)
    cv2.waitKey(0)
    return 0

if __name__ == "__main__":
    main()
