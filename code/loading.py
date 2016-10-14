# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
import math


def isPointBetweenLines(p, l1, l2):
    return np.dot(p,l1)*np.dot(p,l2)*np.dot(l1[0:2],l2[0:2]) <= 0


def getLaneForPoint(p, lines):
    for i in xrange(len(lines)-1):
        if isPointBetweenLines(p, lines[i], lines[i+1]):
            return i
    return None
    

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

