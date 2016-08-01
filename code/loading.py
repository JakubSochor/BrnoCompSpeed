# -*- coding: utf-8 -*-
from __future__ import division
from utils import *

import numpy as np
import math
import matplotlib.pyplot as plt

def loadCars(carsFile, width, height):
    cars = []
    with open(carsFile, "r") as f:
        lines = f.readlines()
        i = 0
        while i < len(lines):
            l = lines[i].split(";")[0:-1]
            l = map(int, l)
            carId, _, pointsCount, occlusion = l
            points = []
            for j in xrange(pointsCount):
                i += 1
                l = map(float, lines[i].split(";")[1:-1])
                points.append((int(l[0]), l[1]*width, l[2]*height))                
            cars.append({"points": points, "occlusion": occlusion, "carId": carId})
            i += 1
    return cars

          
def loadLines(p, width, height):
    lines = []
    with open(p, "r") as f:
        for l in f.readlines():
            vx, vy, x, y = map(float, l.split(";")[1:-1])
            vx *= width
            x *= width
            y *= height
            vy *= height
            pt1 = np.array([x,y,1])
            pt2 = np.array([x+vx, y+vy, 1])
            l = np.cross(pt1, pt2)
            l = l/np.linalg.norm(l[0:2])
            lines.append(l)
    return lines
    
   
def loadPoints(p, width, height):
    pointsGroups = []
    with open(p, "r") as f:
        lines = map(lambda l: l.strip(), f.readlines())
        i = 0
        while i < len(lines):
            l = lines[i].split(";")[0:-1]
            _, pointsCount = map(int, l)
            points = []
            for j in xrange(pointsCount):
                i += 1
                l = map(float, lines[i].split(";")[1:-1])
                points.append(np.array([width*l[0], height*l[1], 1]))
            pointsGroups.append(points)
            i += 1
    return filter(lambda ps: len(ps)>0, pointsGroups)

def isPointBetweenLines(p, l1, l2):
    return np.dot(p,l1)*np.dot(p,l2)*np.dot(l1[0:2],l2[0:2]) <= 0


def getLaneForPoint(p, lines):
    for i in xrange(len(lines)-1):
        if isPointBetweenLines(p, lines[i], lines[i+1]):
            return i
    return None
    
    
def showLanes(laneDivLines, img):
    imgs = []
    for i in xrange(len(laneDivLines)-1):
        imgs.append(img.copy())
    for y in xrange(img.shape[0]):
        for x in xrange(img.shape[1]):
            lane = getLaneForPoint(np.array([x,y,1]), laneDivLines)
            if lane is not None:
                imgs[lane][y,x, :] = np.array([0,255,0], dtype=np.uint8)
    for laneId, im in enumerate(imgs):
        scale = float(10)/img.shape[1]
        plt.figure(figsize = (im.shape[1]*scale, im.shape[0]*scale))
        plt.imshow(im)
        plt.title("Lane %d"%(laneId))
    plt.show()


def getFocal(vp1, vp2, pp):
    return math.sqrt(- np.dot(vp1[0:2]-pp[0:2], vp2[0:2]-pp[0:2]))


"""
Compute camera calibration from two van points and principal point
"""
def computeCameraCalibration(_vp1, _vp2, _pp):
    vp1 = np.concatenate((_vp1, [1]))    
    vp2 = np.concatenate((_vp2, [1]))    
    pp = np.concatenate((_pp, [1]))    
    focal = getFocal(vp1, vp2, pp)
    vp1W = np.concatenate((_vp1, [focal]))    
    vp2W = np.concatenate((_vp2, [focal]))    
    ppW = np.concatenate((_pp, [0])) 
    vp3W = np.cross(vp1W-ppW, vp2W-ppW)
    vp3 = np.concatenate((vp3W[0:2]/vp3W[2]*focal + ppW[0:2], [1]))
    vp3Direction = np.concatenate((vp3[0:2], [focal]))-ppW
    roadPlane = np.concatenate((vp3Direction/np.linalg.norm(vp3Direction), [10]))
    return vp1, vp2, vp3, pp, roadPlane, focal
    
    
"""
Project point p on roadPlane
"""
def getWorldCoordinagesOnRoadPlane(p, focal, roadPlane, pp):
    p = p/p[2]
    pp = pp/pp[2]
    ppW = np.concatenate((pp[0:2], [0]))
    pW = np.concatenate((p[0:2], [focal]))
    dirVec = pW - ppW
    t = -np.dot(roadPlane, np.concatenate((ppW, [1])))/np.dot(roadPlane[0:3], dirVec)
    return ppW + t*dirVec

