# +-------------------------------------------------------------------------
# | ysMotionUtil.py
# |
# | Author: Yoonsang Lee
# +-------------------------------------------------------------------------
# | COPYRIGHT:
# |    Copyright Yoonsang Lee 2013
# |    See the included COPYRIGHT.txt file for further details.
# |    
# |    This file is part of the DataDrivenBipedController.
# |    DataDrivenBipedController is free software: you can redistribute it and/or modify
# |    it under the terms of the MIT License.
# |
# |    You should have received a copy of the MIT License
# |    along with DataDrivenBipedController.  If not, see <mit-license.org>.
# +-------------------------------------------------------------------------

import copy, math
import numpy as np 

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Motion.ysMotion as ym
import Motion.ysMotionAnalysis as yma
import Math.ysFunctionGraph as yfg
import Math.mmMath as mmMath

#===============================================================================
# # skeleton
#===============================================================================
def getAllParentIndexes(skeleton):
    ls = [None]*skeleton.getElementNum()
    for i in range(skeleton.getElementNum()):
        ls[i] = getParentIndexes(skeleton, i)
    return ls
def getParentIndexes(skeleton, index):
    ls = []
    _addParentIndex(skeleton, index, ls)
    return ls 
def _addParentIndex(skeleton, index, ls):
    parentIndex = skeleton.getParentIndex(index)
    if parentIndex!=None:
        ls.append(parentIndex)
        _addParentIndex(skeleton, parentIndex, ls)

def getAllParentJointIndexes(skeleton):
    ls = [None]*skeleton.getJointNum()
    for i in range(skeleton.getJointNum()):
        ls[i] = getParentJointIndexes(skeleton, i)
    return ls
def getParentJointIndexes(skeleton, jointIndex):
    ls = []
    _addParentJointIndex(skeleton, jointIndex, ls)
    return ls 
def _addParentJointIndex(skeleton, jointIndex, ls):
    parentJointIndex = skeleton.getParentJointIndex(jointIndex)
    if parentJointIndex!=None:
        ls.append(parentJointIndex)
        _addParentJointIndex(skeleton, parentJointIndex, ls)


if __name__=='__main__':
    from fltk import *
    import numpy
    import operator as op
    import Motion.ysMotion as ym
    import Resource.ysMotionLoader as yf
    import Motion.ysMotionConverter as ymc
    import Util.ysMatplotEx as ymp
    import Util.ysPythonEx as ype
    import Util.ysGlHelper as ygh
#    import GUI.ysViewer2 as yv2
    import Renderer.ysRenderer as yr
    import GUI.ysSimpleViewer as ysv
    from pylab import *
    import copy
    
    def test_getAllParentIndexes(): 
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        skeleton = motion[0].skeleton
    
        print 'getAllParentIndexes'
        masks = getAllParentIndexes(skeleton)
        print masks
        for i in range(len(masks)):
            print 'parents of', skeleton.getElementName(i), ':',
            for parent in masks[i]:
                print skeleton.getElementName(parent),
            print
        print
    
        print 'getAllParentJointIndexes'
        masks = getAllParentJointIndexes(skeleton)
        print masks
        for i in range(len(masks)):
            print 'parents of', skeleton.getJointName(i), ':',
            for parent in masks[i]:
                print skeleton.getJointName(parent),
            print
        print
        
            
    pass
    test_getAllParentIndexes()
