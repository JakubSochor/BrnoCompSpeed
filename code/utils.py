# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
import os
import matplotlib.pyplot as plt
import cPickle


def ensureDir(d):
    if len(d) > 0:
        if not os.path.exists(d):
            try:
                os.makedirs(d)
            except OSError as e:
                if e.errno != 17: # FILE EXISTS
                    raise e        
                
def loadCache(cacheFile):
    with open(cacheFile, 'rb') as fid:
        return cPickle.load(fid)

def saveCache(cacheFile, data):
    ensureDir(os.path.dirname(cacheFile))
    with open(cacheFile, 'wb') as fid:
        cPickle.dump(data, fid, cPickle.HIGHEST_PROTOCOL)
        
def pointToLineDistance(p, l):
    return abs(np.dot(l,p/p[2]))/np.linalg.norm(l[0:2])
    
def pointToLineProjection(l, p):
    p = p/p[-1]
    c = p[0]*l[1] - p[1]*l[0]
    perpendicularLine = np.array([-l[1], l[0], c])
    intersection = np.cross(l, perpendicularLine)
    return intersection/intersection[-1]
    

def drawLinePyplot(l, *args, **kwargs):
    l = l/np.linalg.norm(l[0:2])
    xlim = plt.xlim()
    ylim = plt.ylim()
    if abs(l[1]) > 0.5:
        pt1 = (xlim[0], (-l[0]*xlim[0]-l[2])/l[1])
        pt2 = (xlim[1], (-l[0]*xlim[1]-l[2])/l[1])
    else:
        pt1 = ((l[1]*ylim[0]+l[2])/(-l[0]), ylim[0])
        pt2 = ((l[1]*ylim[1]+l[2])/(-l[0]), ylim[1])
    plt.plot([pt1[0], pt2[0]], [pt1[1],pt2[1]], *args, **kwargs)        
    plt.xlim(xlim[0], xlim[1])
    plt.ylim(ylim[0], ylim[1])

