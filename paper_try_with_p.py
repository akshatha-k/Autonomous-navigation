import numpy as np
import cv2
import sys
import math
import statistics
import matplotlib.pyplot as plt
import pandas as pd
from shapely.geometry import LineString
import operator
def main():
    src=cv2.imread('./imagesRGB/RGB01.png')
    #src=cv2.imread('stair4.jpg')
    gray= cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gray',gray)
    #gray=cv2.bitwise_not(gray)
    gray=cv2.Sobel(gray, cv2.CV_8U, 0, 1, ksize=5)
    #cv2.imshow("biwise",gray)
    bw=cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, -2)
    #bw= cv2.inRange(gray,230,255)
    #cv2.imshow('binary', bw)
    horizontal=np.copy(bw)
    cols = horizontal.shape[1]
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
    #cv2.imshow('horizontal',horizontal)
    rhofilter=np.copy(src)
    '''
    lines= cv2.HoughLinesP(horizontal,1,math.pi/180,1, 150, 50)

    cv2.imshow("horizontal", horizontal)
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(src, (x1,y1),(x2,y2),(0,255,0),2)
    '''
    lines=[]
    lineP=cv2.HoughLinesP(horizontal,1,math.pi/120,threshold=10,minLineLength=300, maxLineGap=20)
    #print(lineP)
    for line in lineP:
        for x1,y1,x2,y2 in line:
            cv2.line(src,(x1,y1),(x2,y2),(0,255,0),2)
            theta=math.pi-math.atan2((x2-x1),(y2-y1))
            rho=math.sqrt((((x1-x2)**2)*math.sin(theta)**2)+(((y1-y2)**2)*math.cos(theta)**2))
            lines.append([rho,theta])
            #print(rho,theta)
    cv2.imshow('with',src)

    lines_points = []
    rho_window=11

    x1_arr=[]
    y1_arr=[]
    x2_arr=[]
    y2_arr=[]
    temp_list=[]
    line_arr=[]
    #print(lines.shape)
    if lines is not None:
        lines= np.reshape(lines, [-1,2])
        lineP= np.reshape(lineP, [-1,4])
        lines_sorted=lines[lines[:,0].argsort()]
        lineP_sorted=lineP[lines[:,0].argsort()]
        #print(lineP_sorted)

        pos=0
        for i in range(0,len(lines_sorted)-1):
            if abs(lines_sorted[i][0] - lines_sorted[i+1][0]) <= rho_window:
                line_arr.append(lineP_sorted[i])
            else:
                line_arr.append(lineP_sorted[i])
                if len(line_arr) %2 == 0:
                    index = len(line_arr)//2 -1
                else:
                    index = len(line_arr)//2
                x1_arr.append(line_arr[index][0])
                y1_arr.append(line_arr[index][1])
                x2_arr.append(line_arr[index][2])
                y2_arr.append(line_arr[index][3])
                del temp_list[:]
        i = len(lines_sorted)-1
        line_arr.append(lineP_sorted[i])
        if len(line_arr) %2 == 0:
            index = len(line_arr)//2 -1
        else:
            index = len(line_arr)//2

        x1_arr.append(line_arr[index][0])
        y1_arr.append(line_arr[index][1])
        x2_arr.append(line_arr[index][2])
        y2_arr.append(line_arr[index][3])

        for i in range(len(x1_arr)):
            x1= x1_arr[i]
            y1= y1_arr[i]
            x2= x2_arr[i]
            y2= y2_arr[i]
            lines_points.append([(x1,y1),(x2,y2)])
            cv2.line(rhofilter,(x1,y1),(x2,y2),(0,0,255),2)
            #print(rho,theta)

    cv2.imshow('With filter',rhofilter)
    cv2.waitKey(0)
    return 0

if __name__ == "__main__":
    main()
