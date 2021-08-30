#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import with_statement
import cv_bridge
import rospy
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import sys
import os
import signal

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
WIDTH =640
HEIGHT = 480
OFFSET = 340
GAP = 40
FLAG =False


def img_callback(data):
    global image,Flag
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    Flag=True

def draw_lines(img,lines):
    global OFFSET
    for line in lines:
        x1,y1,x2,y2 = line[0]
        color = (random.randint(0,255),random.randint(0,255),random.randint(0,255))
        img = cv2.line(img,(x1,y1+OFFSET),(x2,y2+OFFSET),color,2)
    return img

def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos+rpos) / 2
    cv2.rectangle(img, (lpos-5,15+offset),(lpos+5,25+offset),(0,255,0),2)
    cv2.rectangle(img, (rpos-5,15+offset),(rpos+5,25+offset),(0,255,0),2)
    cv2.rectangle(img, (center-5,15+offset),(center+5,25+offset),(0,255,0),2)
    cv2.rectangle(img, (315,15+offset),(325+5,25+offset),(125,255,125),2)
    return img

def divide_left_right(lines):
    global WIDTH
    low_slope_threshold = 0
    high_slope_threshold = 10
    slopes = []
    new_lines = []
    for line in lines:
        x1,y1,x2,y2 = line[0]
        if x2-x1==0:
            slope =0
        else:
            slope = float(y2-y1) / float(x2-x1)
        if abs(slope)>low_slope_threshold and abs(slope) <high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])
    left_lines =[]
    right_lines =[]
    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]
        x1,y1,x2,y2 = Line
        if slope<0 and x2< WIDTH/2 -90:
            left_lines.append([Line.tolist()])
        elif slope>0 and x1> WIDTH/2 +90:
            right_lines.append([Line.tolist()])
    return left_lines,right_lines

def get_line_params(lines):
    x_sum= 0.0
    y_sum=0.0
    m_sum=0.0
    size = len(lines)
    if size==0:
        return 0,0
    for line in lines:
        x1,y1,x2,y2 = line[0]
        x_sum += x1+x2
        y_sum += y1+y2
        m_sum += float(y2-y1) / float(x2-x1)

    x_avg = x_sum / (size*2)
    y_avg = y_sum / (size*2)
    m = m_sum / (size)
    b = y_avg - m * x_avg
    return m,b

def get_line_pos(img,lines,left=False,right=False):
    global WIDTH, HEIGHT
    global OFFSET, GAP
    m,b = get_line_params(lines)
    if m ==0 and b==0:
        if left:
            pos =0
        if right:
            pos =WIDTH
    else:
        y= GAP /2
        pos =(y-b) /m

        b += OFFSET
        x1 = (HEIGHT - b) /float(m)
        x2 = ((HEIGHT/2) -b) / float(m)

        cv2.line(img, (int(x1), HEIGHT), (int(x2), (HEIGHT/2)), (255, 0,0), 3)
    return img, int(pos)


def process_img(frame):
    global WIDTH
    global OFFSET, GAP
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size,kernel_size),0)

    low_threshold = 200
    high_threshold = 255
    edge_img = cv2.Canny(np.uint8(blur_gray),low_threshold,high_threshold)
    cv2.imshow('canny',edge_img)
    roi = edge_img[OFFSET:OFFSET+GAP, 0:WIDTH]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)
    if all_lines is None:
        print("all_lines is none")
        return 0,640
    
    left_lines, right_lines = divide_left_right(all_lines)
    frame, lpos = get_line_pos(frame, left_lines,left=True)
    frame, rpos = get_line_pos(frame, right_lines,right=True)
    frame = draw_lines(frame,left_lines)
    frame = draw_lines(frame,right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
    frame = draw_rectangle(frame, lpos, rpos, offset=OFFSET)

    cv2.imshow('calibration',frame)
    return lpos, rpos


def start():
    global pub
    global image
    global cap
    global Width,Height

    rospy.init_node('Houglinedetect')
    image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,img_callback)

    print("hi")
    print("-"*10+"Xycar A2 KM v1.0"+"-"*10)
    rospy.sleep(2)
    
    while True:
        if image.size != (640*480*3) and Flag:
            continue
        Flag = False
        lpos,rpos = process_img(image)
        center = (lpos+rpos)/2
        angle = (WIDTH/2 - center)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__=="__main__":
    start()