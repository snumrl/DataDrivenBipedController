# +-------------------------------------------------------------------------
# | ysMotionBlend.py
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
import Motion.ysBipedAnalysis as yba
import Math.ysFunctionGraph as yfg
import Math.mmMath as mm


def align(motionToAlign, alignRefPosture_or_d, alignPosition=True, alignOrientation=True, pos_y_preserve=True, ori_xz_preserve=True, alignFrame=0):
    if alignRefPosture_or_d.__class__ == ym.JointPosture: d = alignRefPosture_or_d - motionToAlign[alignFrame]
    elif alignRefPosture_or_d.__class__ == ym.JointDisplacement: d = alignRefPosture_or_d
        
    if alignPosition:
        p_offset = d.rootPos.copy()
        if pos_y_preserve:
            p_offset[1] = 0.
            d.rootPos[0] = 0.; d.rootPos[2] = 0.
        else:
            d.disableTranslation()
        motionToAlign.translateByOffset(p_offset, False)
        
    if alignOrientation:
#        R_offset = d.getJointOrientationLocal(0)
        R_offset_global = np.dot(alignRefPosture_or_d.localRs[0], motionToAlign[0].localRs[0].T)
        if ori_xz_preserve:
#            R_offset_y, R_offset_xz = mm.projectRotation((0,1,0), R_offset_global)
            R_offset_y, R_offset_xz = mm.projectRotation2((0,1,0), R_offset_global)
            R_offset_global = R_offset_y
#            R_offset_y, R_offset_xz = mm.projectRotation((0,1,0), R_offset)
#            R_offset_y, R_offset_xz = mm.projectRotation(np.dot(alignRefPosture_or_d.localRs[0], (0,1,0)), R_offset)
#            R_offset_y, R_offset_xz = mm.projectRotation2(np.dot(alignRefPosture_or_d.localRs[0].T, (0,1,0)), R_offset)
#            R_offset = R_offset_y
#            d.localRs[0] = np.dot(R_offset_xz, motionToAlign[0].localRs[0])
            d.localRs[0] = R_offset_xz.T
        else:
            d.disableRotations([0])
        
#        motionToAlign.rotateTrajectoryLocal(R_offset)
#        R_offset_global = np.dot(alignRefPosture_or_d.localRs[0], motionToAlign[0].localRs[0].T)
        motionToAlign.rotateTrajectory(R_offset_global)
    
    return d

def align_y_angle(motionToAlign, alignRefPosture_or_d, oriRefPosture, alignPosition=True, alignOrientation=True, pos_y_preserve=True, ori_xz_preserve=True, alignFrame=0):
    if alignRefPosture_or_d.__class__ == ym.JointPosture: d = alignRefPosture_or_d - motionToAlign[alignFrame]
    elif alignRefPosture_or_d.__class__ == ym.JointDisplacement: d = alignRefPosture_or_d
        
    if alignPosition:
        p_offset = d.rootPos.copy()
        if pos_y_preserve:
            p_offset[1] = 0.
            d.rootPos[0] = 0.; d.rootPos[2] = 0.
        else:
            d.disableTranslation()
        motionToAlign.translateByOffset(p_offset, False)
        
    if alignOrientation:
#        R_offset = d.getJointOrientationLocal(0)
        R_offset_global = np.dot(oriRefPosture.localRs[0], motionToAlign[0].localRs[0].T)
        if ori_xz_preserve:
#            R_offset_y, R_offset_xz = mm.projectRotation((0,1,0), R_offset_global)
            R_offset_y, R_offset_xz = mm.projectRotation2((0,1,0), R_offset_global)
            R_offset_global = R_offset_y
#            R_offset_y, R_offset_xz = mm.projectRotation((0,1,0), R_offset)
#            R_offset_y, R_offset_xz = mm.projectRotation(np.dot(alignRefPosture_or_d.localRs[0], (0,1,0)), R_offset)
#            R_offset_y, R_offset_xz = mm.projectRotation2(np.dot(alignRefPosture_or_d.localRs[0].T, (0,1,0)), R_offset)
#            R_offset = R_offset_y
#            d.localRs[0] = np.dot(R_offset_xz, motionToAlign[0].localRs[0])
            d.localRs[0] = R_offset_xz.T
        else:
            d.disableRotations([0])
        
#        motionToAlign.rotateTrajectoryLocal(R_offset)
#        R_offset_global = np.dot(alignRefPosture_or_d.localRs[0], motionToAlign[0].localRs[0].T)
        motionToAlign.rotateTrajectory(R_offset_global)
    
    return d

#===============================================================================
# stitch functions
#===============================================================================

# motion0 : 0000000000    (len 10)
# motion1 : 5555555555    (len 10)
#
# stitch(motion0, motion1, 5, yfg.identity, True) :
# 0000000000
#          5555555555
# =>
#      |   |              -> transitionLength = 5
# 0000001234555555555    len 19, because last frame of motion0 == first frame of motion1
# |   |                  : motion0[:-transitionLength]
#      |   |             : stitchSegment(motion0[-transitionLength:], ...)
#           |       |    : motion1[1:]
#
# stitch(motion0, motion1, 5, yfg.identity, False) :
#          |   |         -> transitionLength = 5
# 0000000000123455555    len 19, because last frame of motion0 == first frame of motion1
def stitch(motion0, motion1, transitionLength, transitionFunc=yfg.identity, editMotion0=False):
    if editMotion0:
        m0 = getStitchedPrevMotion(motion0, motion1[0], transitionLength, transitionFunc)
        m1 = motion1.copy()
    else:
        m0 = motion0.copy()
        m1 = getStitchedNextMotion(motion1, motion0[-1], transitionLength, transitionFunc)

    return m0 + m1

# prevMotion       : 0000000000    (len 10)
# nextStartPosture : 5
#
# getStitchedPrevMotion(prevMotion, nextStartPosture, 5, yfg.identity) :
#     |   |    -> transitionLength = 5
# 000001234    len 9; because prevMotion[-1] should be same with nextStartPosture, so delete last frame of returned motion
def getStitchedPrevMotion(prevMotion, nextStartPosture_d, transitionLength, transitionFunc=yfg.identity, alignPosition=True, alignOrientation=True, pos_y_preserve=True, ori_xz_preserve=True):
    newPrevMotion = prevMotion.copy()
    d = align(newPrevMotion, nextStartPosture_d, alignPosition, alignOrientation, pos_y_preserve, ori_xz_preserve, -1)    
    
    part0 = newPrevMotion[:-transitionLength]
    part1 = stitchSegment(newPrevMotion[-transitionLength:], transitionFunc, d, True)
    
    del part1[-1]
    return part0 + part1

# prevEndPosture : 0
# nextMotion     : 5555555555    (len 10)
#
# getStitchedNextMotion(nextMotion, prevEndPosture, 5, yfg.identity) :
# |   |        -> transitionLength = 5
# 123455555    len 9; because nextMotion[0] should be same with prevEndPosture, so delete first frame of returned motion
def getStitchedNextMotion(nextMotion, prevEndPosture_or_d, transitionLength, transitionFunc=lambda x:1.-yfg.identity(x), alignPosition=True, alignOrientation=True, pos_y_preserve=True, ori_xz_preserve=True):
    newNextMotion = nextMotion.copy()
    d = align(newNextMotion, prevEndPosture_or_d, alignPosition, alignOrientation, pos_y_preserve, ori_xz_preserve)

    part1 = stitchSegment(newNextMotion[:transitionLength], transitionFunc, d, False)
    part2 = newNextMotion[transitionLength:]
    
    del part1[0]
    return part1 + part2

# y_angle < 0 ; left turn
def getStitchedNextMotion_y_angle(nextMotion, prevEndPosture_or_d, prevStartPosture, y_angle, transitionLength, transitionFunc=lambda x:1.-yfg.identity(x)):
    newNextMotion = nextMotion.copy()
    d = align_y_angle(newNextMotion, prevEndPosture_or_d, prevStartPosture, alignPosition=True, alignOrientation=True, pos_y_preserve=True, ori_xz_preserve=True)
#    d.localRs[0] = np.dot(d.localRs[0], mm.rotY(-y_angle))
    newNextMotion.rotateTrajectory(mm.rotY(y_angle))

    part1 = stitchSegment(newNextMotion[:transitionLength], transitionFunc, d, False)
    part2 = newNextMotion[transitionLength:]
    
    del part1[0]
    return part1 + part2

def getAttachedNextMotion(nextMotion, prevEndPosture_or_d, alignPosition=True, alignOrientation=True, pos_y_preserve=True, ori_xz_preserve=True):
    newNextMotion = nextMotion.copy()
    d = align(newNextMotion, prevEndPosture_or_d, alignPosition, alignOrientation, pos_y_preserve, ori_xz_preserve)

    del newNextMotion[0]
    return newNextMotion

# prevEndPosture : 0
# motion(next) : 55555 
# d : -5 (=0-5)
# transitionFunc = mirror(yfg.identity, .5)
# lastMatch = False
# t :      1. .8 .6 .4 .2
# return :  0  1  2  3  4 (motion[0]==prevEndPosture because lastMatch==False)
#
# nextStartPosture : 5
# motion(prev) : 00000
# d : 5 (=5-0)
# transitionFunc = yfg.identity
# lastMatch = True
# t :      .2 .4 .6 .8 1.
# return :  1  2  3  4  5 (motion[-1]==nextStartPosture because lastMatch==True)
def stitchSegment(motion, transitionFunc, d, lastMatch=False):
    scaledTransFunc = yfg.scale(transitionFunc, [0, len(motion)])
    newMotion = ym.JointMotion([None]*len(motion))
    for i in range(len(motion)):
        t = scaledTransFunc(i) if lastMatch==False else scaledTransFunc(i+1)
        newMotion[i] = motion[i] + t*d
#        print i, t
    return newMotion

# motion = 0000000000  (len 10)
# replaceSegment = 55
# replaceFrame = 4 
# transitionLength = 3, transitionFunc = yfg.identity
#         | |
#      | |     
# => 0013553100
# replaceSegment[0] is placed at motion[replaceFrame]
def stitchedReplace(motion, replaceSegment, replaceFrame, transitionLength, transitionFunc=yfg.identity):
    m1 = replaceSegment
    
    startFrame = replaceFrame
    endFrame = replaceFrame + len(replaceSegment) - 1
    
    m0 = motion[:startFrame+1]
    m0 = getStitchedPrevMotion(m0, m1[0], transitionLength, transitionFunc)
    
    m2 = motion[endFrame:]
    m2 = getStitchedNextMotion(m2, m1[-1], transitionLength, transitionFunc)
    
    return m0 + m1 + m2

#===============================================================================
# blend functions
#===============================================================================

#                 | interval0 = [5,11] |
# motion0 : aaaaa          000000       bbbbb
#                 | interval0 = [3,5] |
# motion1 : ccc             22         ddd
#                 | new interval length = 4 |
# blend() : aaaaa           1111             ddd
def blend(motion0, interval0, motion1, interval1, t=None):
    part0 = motion0[:interval0[0]+1].copy()
    
    segment0 = motion0[interval0[0]:interval0[1]+1].copy()
    segment1 = motion1[interval1[0]:interval1[1]+1].copy()
    
    offset = segment0[0].rootPos - segment1[0].rootPos
    segment1.translateByOffset(offset)
    
    part1 = blendSegmentSmooth(segment0, segment1)
    
    part2 = motion1[interval1[1]:].copy()
    part2.translateByOffset(offset)
    
    return part0 + part1 + part2

# don't use this!!!
# t=None : blendSegmentSmooth
# t=0.~1. : blendSegmentFixed
def getBlendedNextMotion2(nextMotionA, nextMotionB, prevEndPosture, t=None, attachPosition=True, attachOrientation=True):
     
    dA = prevEndPosture - nextMotionA[0]
    dB = prevEndPosture - nextMotionB[0]
    
    newNextMotionA = nextMotionA.copy()
    newNextMotionB = nextMotionB.copy()

    if attachPosition:
        p_offset_A = dA.rootPos
        p_offset_B = dB.rootPos
#        d.disableTranslation()
        newNextMotionA.translateByOffset(p_offset_A)
        newNextMotionB.translateByOffset(p_offset_B)

    if attachOrientation:
        R_offset_A = dA.getJointOrientationLocal(0)
        R_offset_A = mm.exp(mm.projectionOnVector(mm.logSO3(R_offset_A), mm.v3(0,1,0))) # # project on y axis
        R_offset_B = dA.getJointOrientationLocal(0)
        R_offset_B = mm.exp(mm.projectionOnVector(mm.logSO3(R_offset_B), mm.v3(0,1,0))) # # project on y axis
#        d.disableRotations([0])
        newNextMotionA.rotateTrajectory(R_offset_A)
        newNextMotionB.rotateTrajectory(R_offset_B)

    if t==None:
        blendedNextMotion = blendSegmentSmooth(newNextMotionA, newNextMotionB)
    else:
        blendedNextMotion = blendSegmentFixed(newNextMotionA, newNextMotionB, t)
    
#    del blendedNextMotion[0]
    return blendedNextMotion



    
def blendSegmentSmooth(motionSegment0, motionSegment1, attachPosition=True, attachOrientation=True):
    motionSegment1 = motionSegment1.copy()
    if attachPosition:
        p_offset = motionSegment0[0].rootPos - motionSegment1[0].rootPos
        motionSegment1.translateByOffset(p_offset)
    if attachOrientation:
        R_offset = np.dot(motionSegment0[0].localRs[0], motionSegment1[0].localRs[0].T)
        R_offset = mm.exp(mm.projectionOnVector(mm.logSO3(R_offset), mm.v3(0,1,0))) # # project on y axis
        motionSegment1.rotateTrajectory(R_offset)
    
    newMotion = ym.JointMotion( [None]*(int( (len(motionSegment0)+len(motionSegment1))/2.) ) )
#    newMotion = ym.JointMotion( [None]*(int( t*len(motionSegment0) + (1-t)*len(motionSegment1)) ) )
    df0 = float(len(newMotion)) / len(motionSegment0)
    df1 = float(len(newMotion)) / len(motionSegment1)
    for frame in range(len(newMotion)):
        normalizedFrame = float(frame)/(len(newMotion)-1)
        normalizedFrame2 = yfg.H1(normalizedFrame)
        normalizedFrame2 += df0*yfg.H2(normalizedFrame)
        normalizedFrame2 += df1*yfg.H3(normalizedFrame)
        
        posture0_at_normalizedFrame = motionSegment0.getPostureAt(normalizedFrame2*(len(motionSegment0)-1))
        posture1_at_normalizedFrame = motionSegment1.getPostureAt(normalizedFrame2*(len(motionSegment1)-1))
        newMotion[frame] = posture0_at_normalizedFrame.blendPosture(posture1_at_normalizedFrame, normalizedFrame2)
    return newMotion
    
def blendSegmentFixed(motionSegment0, motionSegment1, t, attachPosition=True, attachOrientation=True):
    motionSegment1 = motionSegment1.copy()
    if attachPosition:
        p_offset = motionSegment0[0].rootPos - motionSegment1[0].rootPos
        motionSegment1.translateByOffset(p_offset)
    if attachOrientation:
        R_offset = np.dot(motionSegment0[0].localRs[0], motionSegment1[0].localRs[0].T)
        R_offset = mm.exp(mm.projectionOnVector(mm.logSO3(R_offset), mm.v3(0,1,0))) # # project on y axis
        motionSegment1.rotateTrajectory(R_offset)
    
#    newMotion = ym.JointMotion( [None]*(int( (len(motionSegment    0)+len(motionSegment1))/2.) ) )
    newMotion = ym.JointMotion( [None]*(int( (1-t)*len(motionSegment0) + t*len(motionSegment1)) ) )
#    df0 = float(len(newMotion)) / len(motionSegment0)
#    df1 = float(len(newMotion)) / len(motionSegment1)
    for frame in range(len(newMotion)):
        normalizedFrame = float(frame)/(len(newMotion)-1)
#        normalizedFrame2 = yfg.H1(normalizedFrame)
#        normalizedFrame2 += df0*yfg.H2(normalizedFrame)
#        normalizedFrame2 += df1*yfg.H3(normalizedFrame)
#        posture0_at_normalizedFrame = motionSegment0.getPostureAt(normalizedFrame2*(len(motionSegment0)-1))
#        posture1_at_normalizedFrame = motionSegment1.getPostureAt(normalizedFrame2*(len(motionSegment1)-1))
#        newMotion[frame] = posture0_at_normalizedFrame.blendPosture(posture1_at_normalizedFrame, normalizedFrame2)
#        print normalizedFrame*(len(motionSegment0)-1)
        posture0_at_normalizedFrame = motionSegment0.getPostureAt(normalizedFrame*(len(motionSegment0)-1))
        posture1_at_normalizedFrame = motionSegment1.getPostureAt(normalizedFrame*(len(motionSegment1)-1))
#        newMotion[frame] = posture0_at_normalizedFrame.blendPosture(posture1_at_normalizedFrame, normalizedFrame)
        newMotion[frame] = posture0_at_normalizedFrame.blendPosture(posture1_at_normalizedFrame, t)
    return newMotion
    
def timescale(motion, newLength, scalingFunc=yfg.identity):
    if newLength == len(motion):
        return motion
    else:
        newMotion = ym.JointMotion([None]*newLength)
        df0 = float(newLength) / len(motion)
        df1 = float(newLength) / (2*newLength - len(motion))
        for frame in range(len(newMotion)):
            normalizedFrame = float(frame)/(newLength-1)
            if scalingFunc == None:
                normalizedFrame2 = yfg.H1(normalizedFrame)
                normalizedFrame2 += df0*yfg.H2(normalizedFrame)
                normalizedFrame2 += df1*yfg.H3(normalizedFrame)
            else:
                normalizedFrame2 = scalingFunc(normalizedFrame)
            newMotion[frame] = motion.getPostureAt(normalizedFrame2*(len(motion)-1)) 
        return newMotion



if __name__=='__main__':
    import psyco; psyco.full()
    from fltk import *
    import numpy, copy, cProfile, os
    import operator as op
    import Motion.ysMotion as ym
    import Resource.ysMotionLoader as yf
    import Motion.ysMotionConverter as ymc
    import Motion.ysMotionExtend as ymt
    import Util.ysMatplotEx as ymp
    import Util.ysPythonEx as ype
    import Util.ysGlHelper as ygh
    import Renderer.ysRenderer as yr
    import GUI.ysSimpleViewer as ysv
    from pylab import *
    from datetime import datetime

    def test_timescale():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)
        frameTime = 1/30.

        scaledMotion = timescale(jointMotion, 400)
#        scaledMotion = timescale(jointMotion, 400, None)
        print len(jointMotion), len(scaledMotion)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('jointMotion', yr.JointMotionRenderer(jointMotion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('jointMotion', jointMotion)
        viewer.doc.addRenderer('scaledMotion', yr.JointMotionRenderer(scaledMotion, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('scaledMotion', scaledMotion)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()

    def test_blendSegment_time_warping():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)
        frameTime = 1/30.

        firstMotion = jointMotion[0:70]
        
        midMotionSmooth1 = firstMotion + blendSegmentSmooth(jointMotion[70:120], timescale(jointMotion[70:120], len(jointMotion[70:120])*2))
        midMotionFixed1 = firstMotion + blendSegmentFixed(jointMotion[70:120], timescale(jointMotion[70:120], len(jointMotion[70:120])*2), .5)
        midMotionFixed2 = firstMotion + blendSegmentFixed(jointMotion[70:120], timescale(jointMotion[70:120], len(jointMotion[70:120])*2), 0.)
        midMotion = midMotionFixed1
        
        secondMotion = midMotion + timescale(jointMotion[120:], len(jointMotion[120:])*2)
#        print len(midMotionSmooth1), len(midMotionSmooth2)
#        print len(midMotionFixed1), len(midMotionFixed2)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('jointMotion', yr.JointMotionRenderer(jointMotion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('jointMotion', jointMotion)
        viewer.doc.addRenderer('firstMotion', yr.JointMotionRenderer(firstMotion, (255,0,0), yr.LINK_LINE))
        viewer.doc.addObject('firstMotion', firstMotion)
#        viewer.doc.addRenderer('midMotionSmooth1', yr.JointMotionRenderer(midMotionSmooth1, (255,255,0), yr.LINK_LINE))
#        viewer.doc.addObject('midMotionSmooth1', midMotionSmooth1)
##        viewer.doc.addRenderer('midMotionSmooth2', yr.JointMotionRenderer(midMotionSmooth2, (255,255,0), yr.LINK_LINE))
##        viewer.doc.addObject('midMotionSmooth2', midMotionSmooth2)
        viewer.doc.addRenderer('midMotionFixed1', yr.JointMotionRenderer(midMotionFixed1, (255,0,255), yr.LINK_LINE))
        viewer.doc.addObject('midMotionFixed1', midMotionFixed1)
        viewer.doc.addRenderer('midMotionFixed2', yr.JointMotionRenderer(midMotionFixed2, (255,0,255), yr.LINK_LINE))
        viewer.doc.addObject('midMotionFixed2', midMotionFixed2)
        viewer.doc.addRenderer('secondMotion', yr.JointMotionRenderer(secondMotion, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('secondMotion', secondMotion)
        
        viewer.startTimer(frameTime/1.4)
        viewer.show()
        
        Fl.run()
         
    def test_blendedSegment_posture_warping():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion0 = yf.readBvhFile(bvhFilePath, .01)  
        bvhFilePath = '../samples/wd2_WalkSukiko00.bvh' 
#        bvhFilePath = '../samples/wd2_WalkForwardVFast00.bvh'
        motion1 = yf.readBvhFile(bvhFilePath, .01)  
        frameTime = 1/30.

        firstMotion = motion0[0:75]
        
#        midMotionSmooth1 = firstMotion + getBlendedNextMotion2(motion0[75:95], motion1[120:150], firstMotion[-1], None)
#        midMotionFixed1 = firstMotion + getBlendedNextMotion2(motion0[75:95], motion1[120:150], firstMotion[-1], .5)
#        midMotionFixed2 = firstMotion + getBlendedNextMotion2(motion0[75:95], motion1[120:150], firstMotion[-1], 1.)
        midMotionSmooth1 = firstMotion + getAttachedNextMotion(blendSegmentSmooth(motion0[75-1:95], motion1[120-1:150]), firstMotion[-1])
        midMotionFixed1 = firstMotion + getAttachedNextMotion(blendSegmentFixed(motion0[75-1:95], motion1[120-1:150], .5), firstMotion[-1])
        midMotionFixed2 = firstMotion + getAttachedNextMotion(blendSegmentFixed(motion0[75-1:95], motion1[120-1:150], 1.), firstMotion[-1])
        midMotion = midMotionSmooth1
        
        secondMotion = midMotion + getAttachedNextMotion(motion1[150:], midMotion[-1])
#        print len(midMotionSmooth1), len(midMotionSmooth2)
#        print len(midMotionFixed1), len(midMotionFixed2)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
#        viewer.doc.addRenderer('motion0', yr.JointMotionRenderer(motion0, (0,0,255), yr.LINK_LINE))
#        viewer.doc.addObject('motion0', motion0)
#        viewer.doc.addRenderer('motion1', yr.JointMotionRenderer(motion1, (0,0,255), yr.LINK_LINE))
#        viewer.doc.addObject('motion1', motion1)
        viewer.doc.addRenderer('firstMotion', yr.JointMotionRenderer(firstMotion, (255,0,0), yr.LINK_LINE))
        viewer.doc.addObject('firstMotion', firstMotion)
        viewer.doc.addRenderer('midMotionSmooth1', yr.JointMotionRenderer(midMotionSmooth1, (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('midMotionSmooth1', midMotionSmooth1)
        viewer.doc.addRenderer('midMotionFixed1', yr.JointMotionRenderer(midMotionFixed1, (255,0,255), yr.LINK_LINE))
        viewer.doc.addObject('midMotionFixed1', midMotionFixed1)
        viewer.doc.addRenderer('midMotionFixed2', yr.JointMotionRenderer(midMotionFixed2, (0,255,255), yr.LINK_LINE))
        viewer.doc.addObject('midMotionFixed2', midMotionFixed2)
#        viewer.doc.addRenderer('secondMotion', yr.JointMotionRenderer(secondMotion, (0,255,0), yr.LINK_LINE))
#        viewer.doc.addObject('secondMotion', secondMotion)
#        
        viewer.startTimer(frameTime/1.4)
        viewer.show()
        
        Fl.run()
         
    def test_blendedSegment():
        frameTime = 1/30.
        
        # source motions
#        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
#        motion0 = yf.readBvhFile(bvhFilePath, .01)
#        motion1 = motion0.copy()
#        motion1.rotateTrajectory(mm.rotY(math.pi/2.))
#        segment0 = motion0[55:95]
#        segment1 = motion1[55:95].copy()

##        blendedS = blendSegmentSmooth(segment0, segment1, True, True)
##        blendedF = blendSegmentFixed(segment0, segment1, .5, True, True)
#        blendedS = blendSegmentSmooth(segment0, segment1, True, False)
#        blendedF = blendSegmentFixed(segment0, segment1, .5, True, False)
##        blendedS = blendSegmentSmooth(segment0, segment1, False, True)
##        blendedF = blendSegmentFixed(segment0, segment1, .5, False, True)
##        blendedS = blendSegmentSmooth(segment0, segment1, False, False)
##        blendedF = blendSegmentFixed(segment0, segment1, .5, False, False)
        

        bvhFilePath = '../samples/woddy2_walk_straight_fast.bvh'
        motion0 = yf.readBvhFile(bvhFilePath, .01)
        bvhFilePath = '../samples/woddy2_walk_leftfoot_turn_normal.bvh'
        motion1 = yf.readBvhFile(bvhFilePath, .01)
        segment0 = motion1[56:100]
        segment1 = motion1[83:125]

        blendedS = blendSegmentSmooth(segment0, segment1)
        blendedF = blendSegmentFixed(segment0, segment1, .5)

        viewer = ysv.SimpleViewer()
        viewer.record(False)
#        viewer.doc.addRenderer('motion0', yr.JointMotionRenderer(motion0, (0,0,255), yr.LINK_BONE))
#        viewer.doc.addObject('motion0', motion0)
#        viewer.doc.addRenderer('motion1', yr.JointMotionRenderer(motion1, (0,0,255), yr.LINK_BONE))
#        viewer.doc.addObject('motion1', motion1)
        viewer.doc.addRenderer('segment0', yr.JointMotionRenderer(segment0, (0,0,255), yr.LINK_BONE))
        viewer.doc.addObject('segment0', segment0)
        viewer.doc.addRenderer('segment1', yr.JointMotionRenderer(segment1, (0,0,255), yr.LINK_BONE))
        viewer.doc.addObject('segment1', segment1)
        viewer.doc.addRenderer('blendedS', yr.JointMotionRenderer(blendedS, (255,0,0), yr.LINK_BONE))
        viewer.doc.addObject('blendedS', blendedS)
        viewer.doc.addRenderer('blendedF', yr.JointMotionRenderer(blendedF, (0,255,0), yr.LINK_BONE))
        viewer.doc.addObject('blendedF', blendedF)
#        
        viewer.startTimer(frameTime/1.4)
        viewer.show()
        
        Fl.run()
         
    def test_blend():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion0 = yf.readBvhFile(bvhFilePath, .01)
#        bvhFilePath = '../samples/wd2_WalkSukiko00.bvh'
        bvhFilePath = '../samples/wd2_WalkForwardVFast00.bvh'
        motion1 = yf.readBvhFile(bvhFilePath, .01)
        
        hRef = .1; vRef = .3
        RHEEL = motion0[0].skeleton.getElementIndex('RightFoot')
         
        rc0 = yma.getElementContactStates(motion0, RHEEL, hRef, vRef)
        rc1 = yma.getElementContactStates(motion1, RHEEL, hRef, vRef)
        intervals0 = yma.states2intervals(rc0)[0]
        intervals1 = yma.states2intervals(rc1)[0]
        
        blendedMotion = blend(motion0, yma.intIntervalInner(intervals0[3]), motion1, yma.intIntervalInner(intervals1[3]))
       

        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion0', yr.JointMotionRenderer(motion0, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion0', motion0)
        viewer.doc.addRenderer('motion1', yr.JointMotionRenderer(motion1, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion1', motion1)
        viewer.doc.addRenderer('blendedMotion', yr.JointMotionRenderer(blendedMotion, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('blendedMotion', blendedMotion)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()

    def test_stitch():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion0 = yf.readBvhFile(bvhFilePath, .01)
        bvhFilePath = '../samples/wd2_WalkForwardVFast00.bvh'
        motion1 = yf.readBvhFile(bvhFilePath, .01)
        
        hRef = .1; vRef = .3
        LHEEL = motion0[0].skeleton.getElementIndex('LeftFoot')
        RHEEL = motion0[0].skeleton.getElementIndex('RightFoot')
         
        lc0 = yma.getElementContactStates(motion0, LHEEL, hRef, vRef)
        rc0 = yma.getElementContactStates(motion0, RHEEL, hRef, vRef)
        lc1 = yma.getElementContactStates(motion1, LHEEL, hRef, vRef)
        rc1 = yma.getElementContactStates(motion1, RHEEL, hRef, vRef)

        intervals0, states0 = yba.getBipedGaitIntervals(lc0, rc0, 10, .1)
        intervals1, states1 = yba.getBipedGaitIntervals(lc1, rc1, 10, .1)
        print 'wd2_WalkSameSame00'
        print intervals0
        print [yba.GaitState.text[state] for state in states0]
        print 'wd2_WalkForwardVFast00'
        print intervals1
        print [yba.GaitState.text[state] for state in states1]
        print

        seg = 3
        tlen = 10
        
        print 'motion0[:%d] + motion1[%d:]' % (intervals0[seg][1], intervals1[seg][1])
        print 'transition length', tlen
        print 'stitched_0changed', '%d~%d'%(intervals0[seg][1]-tlen, intervals0[seg][1])
        print 'stitched_1changed', '%d~%d'%(intervals0[seg][1], intervals0[seg][1]+tlen)
        print 
        
        just_added = motion0[:intervals0[seg][1]] + motion1[intervals1[seg][1]:]  
        print 'len(just_added)', len(just_added)
        
        stitched_0changed = stitch(motion0[:intervals0[seg][1]], motion1[intervals1[seg][1]-1:], tlen, yfg.identity, True)
        print 'len(stitched_0changed)', len(stitched_0changed)
        
        stitched_1changed = stitch(motion0[:intervals0[seg][1]], motion1[intervals1[seg][1]-1:], tlen, yfg.identity, False)
        print 'len(stitched_1changed)', len(stitched_1changed)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('just_added', yr.JointMotionRenderer(just_added, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('just_added', just_added)
        viewer.doc.addRenderer('stitched_0changed', yr.JointMotionRenderer(stitched_0changed, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('stitched_0changed', stitched_0changed)
        viewer.doc.addRenderer('stitched_1changed', yr.JointMotionRenderer(stitched_1changed, (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('stitched_1changed', stitched_1changed)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()         
            
    def test_getStitchedNextMotion():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)

        lFoot = motion[0].skeleton.getElementIndex('LeftFoot')
        rFoot = motion[0].skeleton.getElementIndex('RightFoot')
        
        hRef = .1; vRef = .3
        lc = yma.getElementContactStates(motion, lFoot, hRef, vRef)
        rc = yma.getElementContactStates(motion, rFoot, hRef, vRef)
        
        steps = yba.getWalkingSteps(lc, rc, True)
        stepMotions = yma.splitMotionIntoSegments(motion, steps)

        motion_stitch = stepMotions[0]
        motion_getStitchedNextMotion = stepMotions[0]
        print len(motion_stitch), len(motion_getStitchedNextMotion)
        
        for i in range(1, len(stepMotions)):
            stepMotions[i] = stepMotions[i][:-5]
            
            motion_stitch = stitch(motion_stitch, stepMotions[i], 5)
            
            stitched = getStitchedNextMotion(stepMotions[i], motion_getStitchedNextMotion[-1], 5)
            motion_getStitchedNextMotion += stitched 

            print len(motion_stitch), len(motion_getStitchedNextMotion)
            
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('motion_stitch', yr.JointMotionRenderer(motion_stitch, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion_stitch', motion_stitch)
        viewer.doc.addRenderer('motion_getStitchedNextMotion', yr.JointMotionRenderer(motion_getStitchedNextMotion, (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion_getStitchedNextMotion', motion_getStitchedNextMotion)
        
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_early_foottouch():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)

        lFoot = motion[0].skeleton.getElementIndex('LeftFoot')
        rFoot = motion[0].skeleton.getElementIndex('RightFoot')
        
        hRef = .1; vRef = .3
        lc = yma.getElementContactStates(motion, lFoot, hRef, vRef)
        rc = yma.getElementContactStates(motion, rFoot, hRef, vRef)
        
        steps = yba.getWalkingSteps(lc, rc, True)
        stepMotions = yma.splitMotionIntoSegments(motion, steps)
        
        earlyCycle = 2
        howEarly = 5
        
        print 'steps[%d] :'%earlyCycle
        print 'motion[%d]~motion[%d] => '%(steps[earlyCycle][0], steps[earlyCycle][1]),
        print 'motion_earlytouch[%d]~motion_earyltouch[%d]'%(steps[earlyCycle][0], steps[earlyCycle][1]-howEarly)

        motion_earlytouch = stepMotions[0]
        for i in range(1, len(stepMotions)):
            if i==earlyCycle:
                stepMotions[i] = stepMotions[i][:-howEarly]
                
#            stitched = getStitchedNextMotion(stepMotions[i], motion_earlytouch[-1], len(stepMotions[i])-1, yfg.halfsine)
            stitched = getStitchedNextMotion(stepMotions[i], motion_earlytouch[-1], len(stepMotions[i])-1, lambda x:1.-yfg.hermite2nd(x))
#            stitched = getStitchedNextMotion(stepMotions[i], motion_earlytouch[-1], len(stepMotions[i])-1, yfg.identity)
            motion_earlytouch += stitched
            
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (100,100,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('motion_earlytouch', yr.JointMotionRenderer(motion_earlytouch, (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion_earlytouch', motion_earlytouch)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_late_foottouch():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)

        lFoot = motion[0].skeleton.getElementIndex('LeftFoot')
        rFoot = motion[0].skeleton.getElementIndex('RightFoot')
        
        hRef = .1; vRef = .3
        lc = yma.getElementContactStates(motion, lFoot, hRef, vRef)
        rc = yma.getElementContactStates(motion, rFoot, hRef, vRef)
        
        steps = yba.getWalkingSteps(lc, rc, True)
        stepMotions = yma.splitMotionIntoSegments(motion, steps)
        
        lateCycle = 2
        howLate = 5
        
        print 'steps[%d] :'%lateCycle
        print 'motion[%d]~motion[%d] => '%(steps[lateCycle][0], steps[lateCycle][1]),
        print 'motion_latetouch[%d]~motion_earyltouch[%d]'%(steps[lateCycle][0], steps[lateCycle][1]+howLate)

        motion_latetouch = stepMotions[0]
        for i in range(1, len(stepMotions)):
            if i==lateCycle:
                stepMotions[i].extend(ymt.extendByIntegration(stepMotions[i], howLate))
                
#            stitched = getStitchedNextMotion(stepMotions[i], motion_latetouch[-1], len(stepMotions[i])-1, yfg.halfsine)
            stitched = getStitchedNextMotion(stepMotions[i], motion_latetouch[-1], len(stepMotions[i])-1, yfg.hermite2nd)
#            stitched = getStitchedNextMotion(stepMotions[i], motion_latetouch[-1], len(stepMotions[i])-1, yfg.identity)
            motion_latetouch += stitched
            
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (100,100,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('motion_latetouch', yr.JointMotionRenderer(motion_latetouch, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion_latetouch', motion_latetouch)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_getStitchedNextMotion2():
        bvhFilePath = '../samples/walk_left_90degree.bvh'
        motion = yf.readBvhFile(bvhFilePath)

        lFoot = motion[0].skeleton.getElementIndex('LeftFoot')
        rFoot = motion[0].skeleton.getElementIndex('RightFoot')
        
        hRef = .1; vRef = .3
        lc = yma.getElementContactStates(motion, lFoot, hRef, vRef)
        rc = yma.getElementContactStates(motion, rFoot, hRef, vRef)
        
        steps = yba.getWalkingSteps(lc, rc, True)
        stepMotions = yma.splitMotionIntoSegments(motion, steps)
        print steps
        
        earlyCycle = 3
        howEarly = 10
        transitionFunc = lambda x:1.-yfg.hermite2nd(x)
        
        print 'steps[%d] :'%earlyCycle
        print 'motion[%d]~motion[%d] => '%(steps[earlyCycle][0], steps[earlyCycle][1]),
        print 'motion_stitched_trans_rot[%d]~motion_stitched_trans_rot[%d]'%(steps[earlyCycle][0], steps[earlyCycle][1]-howEarly)

        motion_stitched_trans = stepMotions[0].copy()
        motion_stitched_trans_rot = stepMotions[0].copy()
        motion_stitched_d = stepMotions[0].copy()
        motion_attached = stepMotions[0].copy()
        for i in range(1, len(stepMotions)):
            if i==earlyCycle:
                stepMotions[i] = stepMotions[i][:-howEarly]
                
#            motion_stitched_trans += getStitchedNextMotion(stepMotions[i], motion_stitched_trans[-1], len(stepMotions[i])-1, transitionFunc, True, False)

            motion_stitched_trans_rot += getStitchedNextMotion(stepMotions[i].copy(), motion_stitched_trans_rot[-1], len(stepMotions[i])-1, transitionFunc, True, True)

#            d = motion_stitched_d[-1] - stepMotions[i][0]            
#            motion_stitched_d += getStitchedNextMotion(stepMotions[i], d, len(stepMotions[i])-1, transitionFunc, True, True)
            
#            motion_attached += getStitchedNextMotion(stepMotions[i], motion_attached[-1], len(stepMotions[i])-1, yfg.zero, True, True)
            motion_attached += getAttachedNextMotion(stepMotions[i], motion_attached[-1], len(stepMotions[i])-1, yfg.zero, True, True)
            
        frame0 = [mm.I_SE3()]
            
        viewer = ysv.SimpleViewer()
        viewer.record(False)
#        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (100,100,255), yr.LINK_LINE))
#        viewer.doc.addObject('motion', motion)
#        viewer.doc.addRenderer('motion_stitched_trans', yr.JointMotionRenderer(motion_stitched_trans, (255,255,0), yr.LINK_LINE))
#        viewer.doc.addObject('motion_stitched_trans', motion_stitched_trans)
        viewer.doc.addRenderer('motion_stitched_trans_rot', yr.JointMotionRenderer(motion_stitched_trans_rot, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion_stitched_trans_rot', motion_stitched_trans_rot)
        viewer.doc.addRenderer('motion_attached', yr.JointMotionRenderer(motion_attached, (255,255,255), yr.LINK_LINE))
        viewer.doc.addObject('motion_attached', motion_attached)
        viewer.doc.addRenderer('frame0', yr.FramesRenderer(frame0, (255,255,0)))
        
        def simulateCallback(frame):
            frame0[0] = motion_stitched_trans_rot[frame].getJointFrame(0)
#            frame0[0] = motion_attached[frame].getJointFrame(0)
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer((1/30.)/1.4)
        viewer.show()
        Fl.run()
      
    def test_getStitchedNextMotion_analysis():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)

        # global
        part1 = motion[:50].copy()
        part2 = motion[49:].copy()
        
        R_offset_orig_global = mm.rotY(math.pi/2)
        part2.rotateTrajectory(R_offset_orig_global)
        print 'R_offset_orig_global =\n', R_offset_orig_global
        
        R_part1 = part1[-1].getJointOrientationLocal(0)
        R_part2 = part2[0].getJointOrientationLocal(0)
        print 'R_part2 =\n', np.dot(R_offset_orig_global, R_part1)
        
        R_offset_global = np.dot(part2[0].localRs[0], part1[-1].localRs[0].T)
        print 'R_offset_global =\n', np.dot(part2[0].localRs[0], part1[-1].localRs[0].T)
        
        part2_attached_global = part2.copy()
        part2_attached_global.translateByOffset((0,0,1))
        part2_attached_global.rotateTrajectory(R_offset_global.T)
        part2_attached_global.translateByOffset((0,0,-1))
        
        # local
        part1 = motion[:50].copy()
        part2 = motion[49:].copy()
        
        R_offset_orig_local= mm.rotY(math.pi/2)
        part2.rotateTrajectoryLocal(R_offset_orig_local)
        print 'R_offset_orig_local =\n', R_offset_orig_local
        
        R_part1 = part1[-1].getJointOrientationLocal(0)
        R_part2 = part2[0].getJointOrientationLocal(0)
        print 'R_part2 =\n', np.dot(R_part1, R_offset_orig_local)
        
        d = part2[0] - part1[-1]    # np.dot(part1[-1].localRs[0].T, part2[0].localRs[0])
        R_offset_local = d.getJointOrientationLocal(0)
        print 'R_offset_local =\n', R_offset_local
        
        part2_attached_local = part2.copy()
        part2_attached_local.translateByOffset((0,0,1))
        part2_attached_local.rotateTrajectoryLocal(R_offset_local.T)
        part2_attached_local.translateByOffset((0,0,-1))
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (100,100,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('part1', yr.JointMotionRenderer(part1, (255,100,255), yr.LINK_LINE))
        viewer.doc.addObject('part1', part1)
        viewer.doc.addRenderer('part2', yr.JointMotionRenderer(part2, (100,255,255), yr.LINK_LINE))
        viewer.doc.addObject('part2', part2)
#        viewer.doc.addRenderer('part1[-1]', yr.JointMotionRenderer(ym.JointMotion([part1[-1]]), (255,100,255), yr.LINK_LINE))
#        viewer.doc.addRenderer('part2[0]', yr.JointMotionRenderer(ym.JointMotion([part2[0]]), (100,255,255), yr.LINK_LINE))
#        viewer.doc.addRenderer('part2_attached_global', yr.JointMotionRenderer(part2_attached_global, (255,0,0), yr.LINK_LINE))
#        viewer.doc.addObject('part2_attached_global', part2_attached_global)
#        viewer.doc.addRenderer('part2_attached_local', yr.JointMotionRenderer(part2_attached_local, (0,255,0), yr.LINK_LINE))
#        viewer.doc.addObject('part2_attached_local', part2_attached_local)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
        
    def test_getStitchedNextMotion_y_angle():
        bvhFilePath = '../samples/walk_left_90degree.bvh'
        motion0 = yf.readBvhFile(bvhFilePath)
        bvhFilePath = '../samples/walk_left_90degree.bvh'
        motion1 = yf.readBvhFile(bvhFilePath)
    
    #    stitched_motion = ymb.stitch(motion0, motion1, 20, yfg.hermite2nd)
        transitionLength = 20
        transitionFunc = lambda x:1.-yfg.hermite2nd(x)
        m0 = motion0.copy()
#        m1 = getStitchedNextMotion(motion1, motion0[-1], transitionLength, transitionFunc, alignOrientation=False)
        m1 = getStitchedNextMotion_y_angle(motion1, motion0[-1], motion0[0], math.pi/4, transitionLength,transitionFunc)
        stitched_motion = m0 + m1
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion0', yr.JointMotionRenderer(motion0, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion0', motion0)
        viewer.doc.addRenderer('motion1', yr.JointMotionRenderer(motion1, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion1', motion1)
        viewer.doc.addRenderer('stitched_motion', yr.JointMotionRenderer(stitched_motion, (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('stitched_motion', stitched_motion)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()         

        
    def profile_stitch_blend():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)

        # global
        part1 = motion[:50].copy()
        part2 = motion[49:].copy()
        
#        motion_stitched_trans_rot += getStitchedNextMotion(stepMotions[i].copy(), motion_stitched_trans_rot[-1], len(stepMotions[i])-1, transitionFunc, True, True)

        profileDataFile = '../samples/profile_stitch_blend_%s.profile'%datetime.today().strftime('%y%m%d_%H%M%S')
#        cProfile.runctx('getStitchedNextMotion(part2, part1[-1], 50)', globals(), locals(), profileDataFile)
        cProfile.runctx('blendSegmentSmooth(part2, part1)', globals(), locals(), profileDataFile)
        os.system('c:\python25\python.exe ../Tools/pprofui.py %s'%profileDataFile)

      
    pass
#    test_blend_graph()
#    test_timescale()
#    test_blendSegment_time_warping()
#    test_blendedSegment_posture_warping()
#    test_blendedSegment()
#    test_blend()
#    test_stitch()

#    test_getStitchedNextMotion()
#    test_early_foottouch()
#    test_late_foottouch()
#    test_getStitchedNextMotion2()
#    test_getStitchedNextMotion_analysis()
    test_getStitchedNextMotion_y_angle()

#    profile_stitch_blend()