# +-------------------------------------------------------------------------
# | main_InteractiveExample2.py
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

from fltk import *
import psyco; psyco.full()
import cPickle, pprint, math, time
from datetime import datetime
import cProfile, os, random

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')
import Math.mmMath as mm
import numpy as np
from collections import deque
import Math.csMath as cm
import Math.ysFunctionGraph as yfg
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Simulator.ysVpUtil as yvu
import GUI.seSimpleViewer_after_submit as ysv
import ArticulatedBody.ysControl as yct
import ArticulatedBody.ysReferencePoints as yrp
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba
import Motion.ysMotion as ym
import Motion.ysMotionBlend as ymb
import Motion.ysMotionExtend as ymt
import Motion.ysSkeletonEdit as yhe
import Motion.mmAnalyticIK as aik 
import Util.ysMatplotEx as ymp
import Resource.ysMotionLoader as yf
import Simulator.ysPhysConfig as ypc
import Simulator.csVpBody as cvb

SIMULATION_ON = True
#SIMULATION_ON = False

#SHOW_MOTION = True
SHOW_MOTION = False

SCREEN_SHOT = False
#SCREEN_SHOT = True

WIREFRAME = True
#WIREFRAME = False
if WIREFRAME:
    polygonStyle = yr.POLYGON_LINE
    linkStyle = yr.LINK_LINE
else:
    polygonStyle = yr.POLYGON_FILL
    linkStyle = yr.LINK_LINE
    
#OBSTACLES = True
OBSTACLES = False

TRANSITION_LENGTH = 10
#TRANSITION_FUNC = lambda x:1.-yfg.identity(x)
TRANSITION_FUNC = lambda x:1.-yfg.hermite2nd(x)

ARROW_HEIGHT = .2
SMALL_FORCE_SIZE = 25.
LARGE_FORCE_SIZE = 50.
FORCE_DURATION = .4

BOX_HEIGHT_NUM = 5
BOX_SIZE = .5
BOX_DENSITY = .5

#BOX0_POS = (3,0,-3)
#BOX0_POS = (12,0,-3)
BOX0_POS = (3,0,2)

#BOX1_POS = (0,0,-10)
#BOX2_POS = (-3,0,2)

WALL_HEIGHT_NUM = 3
#WALL0_POS = (3,0,2)
WALL0_POS = (12,0,-3)

ARROW_FIXED_COLOR = (255,102,0)
ARROW_MOVING_COLOR = (255,183,0)


globalTest = 'before'
def motion_crop_store_another_bvh1():
    print '1'
    
     
def motion_crop_store_another_bvh():
    #slow
     motion1 = yf.readBvhFile('./icmotion_last/woody2_walk_slow_stright.bvh',0.01)
     
     tempFilePath = './icmotion_last/right_left_slow.temp'
     yf.writeBvhFile(tempFilePath,motion1[394:419])
     
     tempFilePath = './icmotion_last/right_right_slow.temp'
     yf.writeBvhFile(tempFilePath,motion1[394:443])
     
     tempFilePath = './icmotion_last/left_left_slow.temp'
     yf.writeBvhFile(tempFilePath,motion1[370:419])
     
     tempFilePath = './icmotion_last/left_right_slow.temp'
     yf.writeBvhFile(tempFilePath,motion1[370:394])
     
#nomal
     motion1 = yf.readBvhFile('./icmotion_last/woody2_walk_normal_stright.bvh',0.01)
     
     tempFilePath = './icmotion_last/right_left_normal.temp'
     yf.writeBvhFile(tempFilePath,motion1[789:809])
     
     tempFilePath = './icmotion_last/right_right_normal.temp'
     yf.writeBvhFile(tempFilePath,motion1[789:826])
     
     tempFilePath = './icmotion_last/left_left_normal.temp'
     yf.writeBvhFile(tempFilePath,motion1[771:809])
     
     tempFilePath = './icmotion_last/left_right_normal.temp'
     yf.writeBvhFile(tempFilePath,motion1[771:789])
     
     tempFilePath = './icmotion_last/stop_left_normal.temp'
     yf.writeBvhFile(tempFilePath,motion1[729:771])
     
     tempFilePath = './icmotion_last/left_right_normal_long.temp'
     yf.writeBvhFile(tempFilePath,motion1[771:826])
     
     tempFilePath = './icmotion_last/right_left_normal_long.temp'
     yf.writeBvhFile(tempFilePath,motion1[789:841])
     
     
   #fast  
     motion1 = yf.readBvhFile('./icmotion_last/woody2_walk_fast_stright.bvh',0.01)
     
     tempFilePath = './icmotion_last/right_left_fast.temp'
     yf.writeBvhFile(tempFilePath,motion1[1219:1233])
     
     tempFilePath = './icmotion_last/right_right_fast.temp'
     yf.writeBvhFile(tempFilePath,motion1[1219:1246])
     
     tempFilePath = './icmotion_last/left_left_fast.temp'
     yf.writeBvhFile(tempFilePath,motion1[1233:1260])
     
     tempFilePath = './icmotion_last/left_right_fast.temp'
     yf.writeBvhFile(tempFilePath,motion1[1233:1246])
     
     #left_turning 45
     motion1 = yf.readBvhFile('./icmotion_last/woody2_walk_normal_left_45.bvh',0.01)
     
     
     tempFilePath = './icmotion_last/left_right_45_leftturning.temp'
     yf.writeBvhFile(tempFilePath,motion1[680:738])
    
     
    #left_turning 90
     motion1 = yf.readBvhFile('./icmotion_last/woody2_walk_normal_left_90.bvh',0.01)
     
     
     tempFilePath = './icmotion_last/left_right_90_leftturning.temp'
     yf.writeBvhFile(tempFilePath,motion1[749:807])
      
      #left_turning 135
     motion1 = yf.readBvhFile('./icmotion_last/woody2_walk_normal_left_135.bvh',0.01)
     
     
     tempFilePath = './icmotion_last/left_right_135_leftturning.temp'
     yf.writeBvhFile(tempFilePath,motion1[815:878])
     

     
     #left_turning 180
     motion1 = yf.readBvhFile('./icmotion_last/woody2_walk_normal_left_180.bvh',0.01)
     
     
     tempFilePath = './icmotion_last/left_right_180_leftturning.temp'
#     yf.writeBvhFile(tempFilePath,motion1[1399:1460])
     yf.writeBvhFile(tempFilePath,motion1[1765:1827])
     
     
     
     ###################################
        #right_turning 45
     motion1 = yf.readBvhFile('./icmotion_last/woddy2_walk_normal_right_45_1.bvh',0.01)
     
     
     tempFilePath = './icmotion_last/right_left_45_rightturning.temp'
     yf.writeBvhFile(tempFilePath,motion1[363:418])
   
     
    #right_turning 90
     motion1 = yf.readBvhFile('./icmotion_last/woddy2_walk_normal_right_90.bvh',0.01)
     
     
     tempFilePath = './icmotion_last/right_left_90_rightturning.temp'
     yf.writeBvhFile(tempFilePath,motion1[420:476])
     

      
      #right_turning 135
     motion1 = yf.readBvhFile('./icmotion_last/woddy2_walk_normal_right_135.bvh',0.01)
     
    
     tempFilePath = './icmotion_last/right_left_135_rightturning.temp'
     yf.writeBvhFile(tempFilePath,motion1[1035:1093])
     
     #right_turning 180
     motion1 = yf.readBvhFile('./icmotion_last/woddy2_walk_normal_right_180.bvh',0.01)
     
    
     tempFilePath = './icmotion_last/right_left_180_rightturning.temp'
#     yf.writeBvhFile(tempFilePath,motion1[1053:1113])
     yf.writeBvhFile(tempFilePath,motion1[1611:1674])
     
     
     #style1
     
     motion1 = yf.readBvhFile('./icmotion_last/woddy2_walk_normal_style_1.bvh',0.01)
     
    
     tempFilePath = './icmotion_last/right_left_style1.temp'
     yf.writeBvhFile(tempFilePath,motion1[130:149])
     
     tempFilePath = './icmotion_last/right_right_style1.temp'
     yf.writeBvhFile(tempFilePath,motion1[130:168])
     
     tempFilePath = './icmotion_last/left_left_style1.temp'
     yf.writeBvhFile(tempFilePath,motion1[110:149])
     
     tempFilePath = './icmotion_last/left_right_style1.temp'
     yf.writeBvhFile(tempFilePath,motion1[110:130])
     
     
   
     
     
     
     
     
     
     ###########################################3
     
     
     
     
     
     
     
     
#     tempFilePath = './rawmotion/left_right_walk_fast_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[444:454])
#     
#     tempFilePath = './rawmotion/right_left_walk_fast_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[434:444])
#     
#     tempFilePath = './rawmotion/left_left_walk_fast_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[444:464])
#    
#     tempFilePath = './rawmotion/right_right_walk_fast_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[434:454])
#     
#     motion1 = yf.readBvhFile('./rawmotion/woddy2_walk_leftfoot_turn_normal.bvh',0.01)
#     
#     tempFilePath = './rawmotion/left_left_walk_90_right_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[421:451])
#    
#     tempFilePath = './rawmotion/left_left_walk_45_right_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[1016:1046])
#     
#     motion1 = yf.readBvhFile('./rawmotion/woddy2_walk_rightfoot_turn_normal.bvh',0.01)
#     
#     tempFilePath = './rawmotion/right_right_walk_90_right_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[2293:2320])
#    
#     tempFilePath = './rawmotion/right_right_walk_45_right_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[1751:1781])
#     
#     motion1 =  yf.readBvhFile('./rawmotion/woddy2_walk_turn_180.bvh',0.01)
#     
#     tempFilePath = './rawmotion/right_left_walk_180_right_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[76:118])
#     
#     tempFilePath = './rawmotion/left_left_walk_180_right_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[62:118])
#     
#     motion1 =  yf.readBvhFile('./rawmotion/woddy2_low_walk.bvh',0.01)
#     
#     tempFilePath = './rawmotion/left_right_low_walk_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[995:1062])
#     
#     tempFilePath = './rawmotion/right_right_low_walk_motion.temp'
#     yf.writeBvhFile(tempFilePath,motion1[979:1062])
#     
     
    
      
     
     
     
     
     
     
     
     print 'end'
     
     
def test_seg():
    
    segfile = open('./icmotion_last/woddy2_walk_fast_stright.seg')
    seginfos = cPickle.load(segfile)
    segfile.close()    
    pprint.pprint(seginfos)


def motion_control(): 
    
    
    tempFilePath = './icmotion_last/right_left_slow.temp'  
    right_left_slow = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/right_right_slow.temp'
    right_right_slow = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_left_slow.temp'
    left_left_slow = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_right_slow.temp'
    left_right_slow = yf.readBvhFile(tempFilePath, 1)
     
#nomal
     
     
    tempFilePath = './icmotion_last/right_left_normal.temp'
    right_left_normal = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/right_right_normal.temp'
    right_right_normal = yf.readBvhFile(tempFilePath, 1)    
     
    tempFilePath = './icmotion_last/left_left_normal.temp'
    left_left_normal = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_right_normal.temp'
    left_right_normal = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/stop_left_normal.temp'
    result_motion = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_right_normal_long.temp'
    left_right_normal_long = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/right_left_normal_long.temp'
    right_left_normal_long = yf.readBvhFile(tempFilePath, 1)
    
     
   #fast  
    
     
    tempFilePath = './icmotion_last/right_left_fast.temp'
    right_left_fast = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/right_right_fast.temp'
    right_right_fast = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_left_fast.temp'
    left_left_fast = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_right_fast.temp'
    left_right_fast = yf.readBvhFile(tempFilePath, 1)
     
     #left_turning 45
    
     
     
    tempFilePath = './icmotion_last/left_right_45_leftturning.temp'
    left_right_45_leftturning = yf.readBvhFile(tempFilePath, 1)
     
    #left_turning 90
 
     
     
    tempFilePath = './icmotion_last/left_right_90_leftturning.temp'
    left_right_90_leftturning = yf.readBvhFile(tempFilePath, 1)
      
      #left_turning 135
    
     
     
    tempFilePath = './icmotion_last/left_right_135_leftturning.temp'
    left_right_135_leftturning = yf.readBvhFile(tempFilePath, 1)

     
     #left_turning 180
    
     
     
    tempFilePath = './icmotion_last/left_right_180_leftturning.temp'
    left_right_180_leftturning = yf.readBvhFile(tempFilePath, 1)
     
     
     
     ###################################
        #right_turning 45
    
     
     
    tempFilePath = './icmotion_last/right_left_45_rightturning.temp'
    right_left_45_rightturning = yf.readBvhFile(tempFilePath, 1)
     
    #right_turning 90
    
     
    tempFilePath = './icmotion_last/right_left_90_rightturning.temp'
    right_left_90_rightturning = yf.readBvhFile(tempFilePath, 1)

      
      #right_turning 135
     
    
    tempFilePath = './icmotion_last/right_left_135_rightturning.temp'
    right_left_135_rightturning = yf.readBvhFile(tempFilePath, 1)
    
     
     #right_turning 180
    
    
    tempFilePath = './icmotion_last/right_left_180_rightturning.temp'
    right_left_180_rightturning = yf.readBvhFile(tempFilePath, 1)
    
    #style1
    tempFilePath = './icmotion_last/right_left_style1.temp'
    right_left_style1 = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/right_right_style1.temp'
    right_right_style1 = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_left_style1.temp'
    left_left_style1 = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_right_style1.temp'
    left_right_style1 = yf.readBvhFile(tempFilePath, 1)
     
     
     #style2
    tempFilePath = './icmotion_last/right_left_style2.temp'
    right_left_style2 = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/right_right_style2.temp'
    right_right_style2 = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_left_style2.temp'
    left_left_style2 = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_right_style2.temp'
    left_right_style2 = yf.readBvhFile(tempFilePath, 1)
     
     #style3
    tempFilePath = './icmotion_last/right_left_style3.temp'
    right_left_style3 = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/right_right_style3.temp'
    right_right_style3 = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_left_style3.temp'
    left_left_style3 = yf.readBvhFile(tempFilePath, 1)
     
    tempFilePath = './icmotion_last/left_right_style3.temp'
    left_right_style3 = yf.readBvhFile(tempFilePath, 1)

    
    #init
    #0 :  left 1 :right
    which_foot = [0]
    
    next_motion = left_right_normal[:]

    start_motion = [left_right_normal[:]]
    angle_offset = [0.]
    
    pre_motion_start = [0]
    pre_motion_start[0] = len(result_motion)
    
    
    total_frame_num = [len(result_motion)]
    
    draw_angle = [0]
    angle = [0]
    store_angle_per_frame = [-1]*100000
    store_speed_per_frame = [-1]*100000
    speed = [0]   
    old_speed = [0]  
    
    cross_vector = [0]
    style = [0]
    old_style = [0]
    
  
    velocity = [None]
    direction =[None]
    
    velocity_root = [None]
    tmp = [None]
    relative_angle = [0]
   
    last_arrow_pt = [None]
    tmp_angle = [0]
    
    pre_velocity = [0]
    #style arrow
    which_style = [0]*100000
    

    root_pt= [None]
    
    
 #    result_motion1 = left_right_180_leftturning.copy()
#    result_motion1 = right_left_normal.copy()
#    result_motion.extend(ymb.getStitchedNextMotion(right_left_style1,result_motion[-1],10))
#    result_motion.extend(ymb.getStitchedNextMotion(left_right_style1,result_motion[-1],10))
#    blend_motion = ymb.getBlendedNextMotion2(left_left_normal, left_left_style1,left_left_normal[0],None) 
#    result_motion1.extend(ymb.getStitchedNextMotion(left_right_180_leftturning,result_motion1[-1],1))
#    result_motion1.extend(ymb.getStitchedNextMotion(right_left_normal,result_motion1[-1],10)) 
    

               
    def call_motionstitch_time(finished_frame):
#        motion_info = [[smooth_blended_file1, smooth_blended_file2, start_t, end_t], [blended_file1, blended_file2, t]]
#                                                                                  or [stitched_file]]
        motion_info = [[], []]
        
#        smooth_file1 = 'adfdsf'
#        motion_info[0] = [smooth_file1,]
#        motion_info[0].append(file1)
        
#        filenames.append('name1')
#        tmp_motion = right_left_fast.copy()
        call_time_frame  = len(result_motion)
        last_time_frame = call_time_frame 
        
        next_angle_offset = 0.;
        
        if style[0] != 0:     
            if style[0] == 1:
                if old_style[0] != style[0]:
                    #need a blending 
                    if which_foot[0] == 0:
                        blend_motion = ymb.getBlendedNextMotion2(left_left_normal, left_left_style1,left_left_normal[0],None) 
                        motion_info[0].append('left_left_normal')
                        motion_info[0].append('left_left_style1')
                        motion_info[1].append('left_right_style1')
                        next_motion[:] = left_right_style1.copy()

                        which_foot[0] =1
                    else :
                        blend_motion = ymb.getBlendedNextMotion2(right_right_normal, right_right_style1,right_right_normal[0],None)     
                        motion_info[0].append('right_right_normal')
                        motion_info[0].append('right_right_style1')
                        motion_info[1].append('right_left_style1')     
                        next_motion[:] = right_left_style1.copy()
                        
                        which_foot[0] =0
                    
#                    result_motion.extend(ymb.getStitchedNextMotion(blend_motion,result_motion[last_time_frame-1 ],10))
                    result_motion.extend(ymb.getStitchedNextMotion_y_angle(blend_motion,result_motion[last_time_frame-1 ],start_motion[0][0], angle_offset[0]*mm.RAD,TRANSITION_LENGTH, TRANSITION_FUNC))
   
                    last_time_frame = len(result_motion)
                else:
                    #just stitch
                     if which_foot[0] == 0:
                        motion_info[1].append('left_right_style1')     
                        next_motion[:] = left_right_style1.copy()
                        which_foot[0] =1
                     else :
                        motion_info[1].append('right_left_style1')    
                        next_motion[:] = right_left_style1.copy()
                        which_foot[0] =0  
            elif style[0] == 2:
                if old_style[0] != style[0]:
                    #need a blending 
                    if which_foot[0] == 0:
                        motion_info[0].append('left_left_normal')
                        motion_info[0].append('left_left_style2')
                        motion_info[1].append('left_right_style1') 
                        blend_motion = ymb.getBlendedNextMotion2(left_left_normal, left_left_style2,left_left_normal[0],None) 
                        next_motion[:] = left_right_style1.copy()
                        which_foot[0] =1
                    else :
                        motion_info[0].append('right_right_normal')
                        motion_info[0].append('right_right_style2')
                        motion_info[1].append('right_left_style1') 
                        blend_motion = ymb.getBlendedNextMotion2(right_right_normal, right_right_style2,right_right_normal[0],None)     
                        next_motion[:] = right_left_style1.copy()
                        which_foot[0] =0
                    
#                    result_motion.extend(ymb.getStitchedNextMotion(blend_motion,result_motion[last_time_frame-1 ],10))
                    result_motion.extend(ymb.getStitchedNextMotion_y_angle(blend_motion,result_motion[last_time_frame-1 ],start_motion[0][0], angle_offset[0]*mm.RAD,TRANSITION_LENGTH, TRANSITION_FUNC))
   
                    last_time_frame = len(result_motion)
                else:
                    #just stitch
                     if which_foot[0] == 0:
                        motion_info[1].append('left_right_style2') 
                        next_motion[:] = left_right_style2.copy()
                        which_foot[0] =1
                     else :
                        motion_info[1].append('right_left_style2') 
                        next_motion[:] = right_left_style2.copy()
                        which_foot[0] =0  
            #arrow_another color_ change      
            elif style[0] == 3:
                if old_style[0] != style[0]:
                    #need a blending 
                    if which_foot[0] == 0:
                        motion_info[0].append('left_left_normal')
                        motion_info[0].append('left_left_style3')
                        motion_info[1].append('left_right_style3') 
                        blend_motion = ymb.getBlendedNextMotion2(left_left_normal, left_left_style3,left_left_normal[0],None) 
                        next_motion[:] = left_right_style3.copy()
                        which_foot[0] =1
                    else :
                        motion_info[0].append('right_right_normal')
                        motion_info[0].append('right_right_style3')
                        motion_info[1].append('right_left_style3') 
                        blend_motion = ymb.getBlendedNextMotion2(right_right_normal, right_right_style3,right_right_normal[0],None)     
                        next_motion[:] = right_left_style3.copy()
                        which_foot[0] =0
                    
#                    result_motion.extend(ymb.getStitchedNextMotion(blend_motion,result_motion[last_time_frame-1 ],10))
                    result_motion.extend(ymb.getStitchedNextMotion_y_angle(blend_motion,result_motion[last_time_frame-1 ],start_motion[0][0], angle_offset[0]*mm.RAD,TRANSITION_LENGTH, TRANSITION_FUNC))
   
                    last_time_frame = len(result_motion)
                else:
                    #just stitch
                     if which_foot[0] == 0:
                        motion_info[1].append('left_right_style3') 
                        next_motion[:] = left_right_style3.copy()
                        which_foot[0] =1
                     else :
                        motion_info[1].append('right_left_style3') 
                        next_motion[:] = right_left_style3.copy()
                        which_foot[0] =0  
                
            
            #no angle change
        else:
    
            if relative_angle[0] == 0:
               
                if old_style[0] != style[0]:
                        #need a blending 
                    if old_style[0] == 1:
                        if which_foot[0] == 0:
                            motion_info[0].append('left_left_style1')
                            motion_info[0].append('left_left_normal')
                        
                            blend_motion = ymb.getBlendedNextMotion2(left_left_style1, left_left_normal,left_left_style1[0],None) 
                        else :
                            motion_info[0].append('right_right_style1')
                            motion_info[0].append('right_right_normal')
                            blend_motion = ymb.getBlendedNextMotion2(right_right_style1, right_right_normal,right_right_style1[0],None)     
#                        result_motion.extend(ymb.getStitchedNextMotion(blend_motion,result_motion[last_time_frame-1],10))   
                        result_motion.extend(ymb.getStitchedNextMotion_y_angle(blend_motion,result_motion[last_time_frame-1 ],start_motion[0][0], angle_offset[0]*mm.RAD,TRANSITION_LENGTH, TRANSITION_FUNC))
                        
                        last_time_frame = len(result_motion)
                    elif old_style[0] == 2:
                        if which_foot[0] == 0:
                            motion_info[0].append('left_left_style2')
                            motion_info[0].append('left_left_normal')
                            blend_motion = ymb.getBlendedNextMotion2(left_left_style2, left_left_normal,left_left_style2[0],None) 
                        else :
                            motion_info[0].append('right_right_style2')
                            motion_info[0].append('right_right_normal')
                            blend_motion = ymb.getBlendedNextMotion2(right_right_style2, right_right_normal,right_right_style2[0],None)     
#                        result_motion.extend(ymb.getStitchedNextMotion(blend_motion,result_motion[last_time_frame-1],10))   
                        result_motion.extend(ymb.getStitchedNextMotion_y_angle(blend_motion,result_motion[last_time_frame-1 ],start_motion[0][0], angle_offset[0]*mm.RAD,TRANSITION_LENGTH, TRANSITION_FUNC))
                        
                        last_time_frame = len(result_motion)
                    elif old_style[0] == 3:
                        if which_foot[0] == 0:
                            motion_info[0].append('left_left_style3')
                            motion_info[0].append('left_left_normal')
                            blend_motion = ymb.getBlendedNextMotion2(left_left_style3, left_left_normal,left_left_style2[0],None) 
                        else :
                            motion_info[0].append('right_right_style3')
                            motion_info[0].append('right_right_normal')
                            blend_motion = ymb.getBlendedNextMotion2(right_right_style3, right_right_normal,right_right_style3[0],None)     
#                        result_motion.extend(ymb.getStitchedNextMotion(blend_motion,result_motion[last_time_frame-1],10))
                        result_motion.extend(ymb.getStitchedNextMotion_y_angle(blend_motion,result_motion[last_time_frame-1 ],start_motion[0][0], angle_offset[0]*mm.RAD,TRANSITION_LENGTH, TRANSITION_FUNC))
                           
                        last_time_frame = len(result_motion) 
                #left_foot
                if which_foot[0] == 0:
                    if old_speed[0] != speed[0]:  
                        blend1 = ymb.getBlendedNextMotion2(left_left_normal, left_left_fast,left_left_normal[0],0.1*old_speed[0]) 
                        blend2 = ymb.getBlendedNextMotion2(left_left_normal, left_left_fast,left_left_normal[0],0.1*speed[0]) 
                        blend_motion = ymb.getBlendedNextMotion2( blend1,blend2,blend1[0],None)
                        motion_info[0].append('left_left_normal')
                        motion_info[0].append('left_left_fast')
                        motion_info[0].append(0.1*old_speed[0])
                        motion_info[0].append(0.1*speed[0])

#                        result_motion.extend(ymb.getStitchedNextMotion(blend_motion,result_motion[last_time_frame-1],10))
                        result_motion.extend(ymb.getStitchedNextMotion_y_angle(blend_motion,result_motion[last_time_frame-1 ],start_motion[0][0], angle_offset[0]*mm.RAD,TRANSITION_LENGTH, TRANSITION_FUNC))   
                        
                        last_time_frame = len(result_motion)
                    if speed[0]== 0 :
                        motion_info[1].append('left_right_normal')
                        next_motion[:]  = left_right_normal.copy()
                    elif speed[0] == 10: 
                        motion_info[1].append('left_right_fast')
                        next_motion[:] = left_right_fast.copy()
                    else:
                        motion_info[1].append('left_right_normal')
                        motion_info[1].append('left_right_fast')
                        motion_info[1].append(0.1*speed[0])
                        next_motion[:] = ymb.getBlendedNextMotion2(left_right_normal, left_right_fast,left_right_normal[0],0.1*speed[0]) 
                    
                    which_foot[0] = 1
#                right_foot
                else:
                    if old_speed[0] != speed[0]:  
                        blend1 = ymb.getBlendedNextMotion2(right_right_normal, right_right_fast,right_right_normal[0],0.1*old_speed[0]) 
                        blend2 = ymb.getBlendedNextMotion2(right_right_normal, right_right_fast,right_right_normal[0],0.1*speed[0]) 
                        blend_motion = ymb.getBlendedNextMotion2( blend1,blend2,blend1[0],None)
#                        tmp_motion  = ymb.getStitchedNextMotion(blend_motion,result_motion[finished_frame-1],10)
                        motion_info[0].append('right_right_normal')
                        motion_info[0].append('right_right_fast')
                        motion_info[0].append(0.1*old_speed[0])
                        motion_info[0].append(0.1*speed[0])
                        
#                        result_motion.extend(ymb.getStitchedNextMotion(blend_motion,result_motion[last_time_frame-1],10))
                        result_motion.extend(ymb.getStitchedNextMotion_y_angle(blend_motion,result_motion[last_time_frame-1 ],start_motion[0][0], angle_offset[0]*mm.RAD,TRANSITION_LENGTH, TRANSITION_FUNC))

                        last_time_frame = len(result_motion)
                    if speed[0]== 0 :
                        motion_info[1].append('right_left_normal')
                        next_motion[:]  = right_left_normal.copy()
                    elif speed[0] == 10:
                        motion_info[1].append('right_left_fast')
                        next_motion[:] = right_left_fast.copy()
                    else:
                        motion_info[1].append('right_left_normal')
                        motion_info[1].append('right_left_fast')
                        motion_info[1].append(0.1*speed[0])
                        next_motion[:] = ymb.getBlendedNextMotion2(right_left_normal, right_left_fast,right_left_normal[0],0.1*speed[0]) 
                   
                    which_foot[0] = 0
                        
                 # angle change
            else:
                
                relative_angle[0] = 0 
                pre_velocity[0]  = velocity[0]
                if cross_vector[0] > 0 :
                      
                    print tmp_angle[0], 'left' 
                    if which_foot[0] == 1:   
#                        result_motion.extend(ymb.getStitchedNextMotion(right_left_normal,result_motion[last_time_frame-1],10))           
                        result_motion.extend(ymb.getStitchedNextMotion(right_left_normal,result_motion[last_time_frame-1],TRANSITION_LENGTH, TRANSITION_FUNC))           
                        last_time_frame = len(result_motion)
                    if tmp_angle[0]  < 90:
                        motion_info[1].append('left_right_normal_long')
                        motion_info[1].append('left_right_90_leftturning')
                        motion_info[1].append((tmp_angle[0])/90.0)
                        next_motion[:] =  ymb.getBlendedNextMotion2(left_right_normal_long,left_right_90_leftturning,left_right_normal_long[0],(tmp_angle[0])/90.0) 
                    elif tmp_angle[0] == 90:
                        motion_info[1].append('left_right_90_leftturning')
                        next_motion[:] = left_right_90_leftturning.copy()
    #                              elif tmp_angle[0] < 135:
    #                                  next_motion[:] =  ymb.getBlendedNextMotion2(left_right_walk_90_left_motion,left_right_walk_135_left_motion,left_right_normal_long[0],(tmp_angle[0])/45.0) 
                    elif tmp_angle[0]  < 135:
                        motion_info[1].append('left_right_90_leftturning')
                        motion_info[1].append('left_right_135_leftturning')
                        motion_info[1].append((tmp_angle[0]-90)/45.0)
                        next_motion[:] =  ymb.getBlendedNextMotion2(left_right_90_leftturning,left_right_135_leftturning,left_right_90_leftturning[0],(tmp_angle[0]-90)/45.0) 
                    elif tmp_angle[0] == 135:
                        motion_info[1].append('left_right_135_leftturning')
                        next_motion[:] = left_right_135_leftturning.copy()
                    elif tmp_angle[0] < 180:
                        motion_info[1].append('left_right_135_leftturning')
                        motion_info[1].append('left_right_180_leftturning')
                        motion_info[1].append((tmp_angle[0]-135)/45.0)
                        next_motion[:] =  ymb.getBlendedNextMotion2(left_right_135_leftturning,left_right_180_leftturning,left_right_135_leftturning[0],(tmp_angle[0]-135)/45.0) 
                    else:
                         motion_info[1].append('left_right_180_leftturning')
                         next_motion[:] = left_right_180_leftturning.copy()
                    which_foot[0] = 1          
                    #left_right_45_leftturning
                    next_angle_offset += tmp_angle[0]

                else:
                    print tmp_angle[0], 'right'
                    if which_foot[0] == 0: 
#                        result_motion.extend(ymb.getStitchedNextMotion(left_right_normal,result_motion[last_time_frame-1],10))           
                        result_motion.extend(ymb.getStitchedNextMotion(left_right_normal,result_motion[last_time_frame-1],TRANSITION_LENGTH, TRANSITION_FUNC))           
                        last_time_frame = len(result_motion)
                    if tmp_angle[0]  < 90:
                        motion_info[1].append('right_left_normal_long')
                        motion_info[1].append('right_left_90_rightturning')
                        motion_info[1].append((tmp_angle[0])/90.0)
                        next_motion[:] =  ymb.getBlendedNextMotion2(right_left_normal_long,right_left_90_rightturning,right_left_normal_long[0],(tmp_angle[0])/90.0) 
                    elif tmp_angle[0] == 90:
                        motion_info[1].append('right_left_90_rightturning')
                        next_motion[:] = right_left_90_rightturning.copy()
    #                              elif tmp_angle[0] < 135:
    #                                  next_motion[:] =  ymb.getBlendedNextMotion2(left_right_walk_90_left_motion,left_right_walk_135_left_motion,left_right_normal_long[0],(tmp_angle[0])/45.0) 
                    elif tmp_angle[0] < 135:
                        motion_info[1].append('right_left_90_rightturning')
                        motion_info[1].append('right_left_135_rightturning')
                        motion_info[1].append((tmp_angle[0]-90)/45.0)
                        next_motion[:] =  ymb.getBlendedNextMotion2(right_left_90_rightturning,right_left_135_rightturning,right_left_90_rightturning[0],(tmp_angle[0]-90)/45.0) 
                    elif tmp_angle[0] == 135:   
                        motion_info[1].append('right_left_135_rightturning')
                        next_motion[:] = right_left_135_rightturning.copy()
                    elif tmp_angle[0] < 180:
                        motion_info[1].append('right_left_135_rightturning')
                        motion_info[1].append('right_left_180_rightturning')
                        motion_info[1].append((tmp_angle[0]-135)/45.0)
                        next_motion[:] =  ymb.getBlendedNextMotion2(right_left_135_rightturning,right_left_180_rightturning,right_left_135_rightturning[0],(tmp_angle[0]-135)/45.0) 
                    else:   
                        motion_info[1].append('right_left_180_rightturning')
                        next_motion[:] = right_left_180_rightturning.copy()      
                    which_foot[0] = 0
                    next_angle_offset += -tmp_angle[0]

                pre_angle[0] = 0           
                viewer.objectInfoWnd.angle_value_input.value(0)
                viewer.objectInfoWnd.angle_value_slider.value(0)
               
#                if draw_angle[0] >= 360:
#                    draw_angle[0] = draw_angle[0] -360          
            
#            tmp_motion.extend(ymb.getStitchedNextMotion(next_motion,tmp_motion[-1],10))
      
#        result_motion.extend(ymb.getStitchedNextMotion(next_motion,result_motion[last_time_frame-1],10))
        result_motion.extend(ymb.getStitchedNextMotion_y_angle(next_motion,result_motion[last_time_frame-1], start_motion[0][0], angle_offset[0]*mm.RAD, 10))
        angle_offset[0] += next_angle_offset
        
        for finished_frame in range(finished_frame,len(result_motion)+1):
#            print style[0]
            which_style[finished_frame-1] = style[0]   
        viewer.setMaxFrame(10000)
#            viewer.setMaxFrame(len(result_motion))
        total_frame_num[0] = len(result_motion)
        old_speed[0] = speed[0]
        old_style[0] = style[0]
        
#        
#        
        return result_motion[call_time_frame:], motion_info
        
        
    pre_angle = [0]
    moving_angle = ['off']
    def style_but_cb(which):
        style[0] = which
    def speed_but_cb(which):
        speed[0] = which
    
    def val_slider_handle(which):
        moving_angle[0] = which
#        print '1: ' ,moving_angle[0]
        if  moving_angle[0]== 'on':
            relative_angle[0] = 0
        elif moving_angle[0]== 'off':
            relative_angle[0] = 1
            
    def angle_but_cb_handle(which):  
   
       
        angle[0] = which
        draw_angle[0] -= pre_angle[0]
        draw_angle[0] += angle[0]
        pre_angle[0] = angle[0]
    

    def angle_but_cb(which):
#        relative_angle[0] = 1
#        angle[0] = which
#        draw_angle[0] += angle[0]
#        if draw_angle[0] >= 360:
#            draw_angle[0] = draw_angle[0] -360
            
          
        if which == 270:
            forceInfos.append(ForceInfo(viewer.getCurrentFrame(), FORCE_DURATION, mm.v3(-FORCE_SIZE[0], 0, 0), spine))
        elif which== 90:
            forceInfos.append(ForceInfo(viewer.getCurrentFrame(), FORCE_DURATION, mm.v3(FORCE_SIZE[0], 0, 0), spine))
        elif which == 180:
            forceInfos.append(ForceInfo(viewer.getCurrentFrame(), FORCE_DURATION, mm.v3(0, 0, -FORCE_SIZE[0]), spine))
        elif which == 0:
            forceInfos.append(ForceInfo(viewer.getCurrentFrame(), FORCE_DURATION, mm.v3(0, 0, FORCE_SIZE[0]), spine))
#        else:
#           
#            relative_angle[0] = 1
#            angle[0] = which
#            draw_angle[0] -= pre_angle[0]
#            draw_angle[0] += angle[0]
#            pre_angle[0] = angle[0]
            
    FORCE_SIZE = [LARGE_FORCE_SIZE]
    x_axis = [0]
    z_axis = [0]
    def event_handle(which_key):
        #6 : speed up 7: speed down
        if which_key == 1:
#            print speed
            speed[0]+= 2
            if speed[0] > 10:
                speed[0] = 0                  
        elif which_key == 2: 
#            print speed
            speed[0]-= 2
            if speed[0] < 0:
                speed[0] = 10  
 

        # external force
        
        elif which_key == 'a':
            x_axis[0] += 0.3
        elif which_key == 'd':
            x_axis[0] -= 0.3
        elif which_key == 'w':
            z_axis[0] += 0.3
        elif which_key == 's':
            z_axis[0] -= 0.3
        elif which_key == 'l':
            
            
            if FORCE_SIZE[0]== LARGE_FORCE_SIZE:
                print '2'
                FORCE_SIZE[0]= SMALL_FORCE_SIZE
#                arrow_force.ratio = 0.01
               
            else:
                print '1'
                FORCE_SIZE[0]=LARGE_FORCE_SIZE
#                arrow_force.ratio = 0.005


           
    def make_motion_seq(now_frame_num):
        if now_frame_num > 0 :
            make_motion_seq_after_0_frame(now_frame_num)               
   
    root_frame = [None]
    def make_motion_seq_after_0_frame(now_frame_num):


        root_frame[0] = result_motion[now_frame_num-1].getJointFrame(0)
        
        if SIMULATION_ON: 
            direction[0] =  controlModel.getJointPositionGlobal(0)
        else:
#            viewer.setCameraTarget(result_motion[now_frame_num-1].getJointPositionGlobal(0))  
            direction[0] = result_motion[now_frame_num-1].getJointPositionGlobal(0)   
       
        viewer.motionViewWnd.glWindow.floor_axis[0] = direction[0]

        arrow_origin = direction[0] 
        
        x = -0.7
        z = 0
    
        if store_angle_per_frame[now_frame_num-1] == -1:  
            store_angle_per_frame[now_frame_num-1] = draw_angle[0]
                    
        x1 = z*math.sin(store_angle_per_frame[now_frame_num-1]*math.pi/180) + x*math.cos(store_angle_per_frame[now_frame_num-1]*math.pi/180)
        z1 = z*math.cos(store_angle_per_frame[now_frame_num-1]*math.pi/180) - x*math.sin(store_angle_per_frame[now_frame_num-1]*math.pi/180)
       
        tmp[0] = [x1,0,z1]
      
        arrow_origin = direction[0] + tmp 
        direction[0] = arrow_origin[0]
        direction[0][1] = ARROW_HEIGHT
        if store_speed_per_frame[now_frame_num-1] == -1:
            store_speed_per_frame[now_frame_num-1] = speed[0]
        
        x = -(store_speed_per_frame[now_frame_num-1]*0.1 + 0.5)
        z = 0
        
        x1 = z*math.sin((store_angle_per_frame[now_frame_num-1])*math.pi/180) + x*math.cos((store_angle_per_frame[now_frame_num-1])*math.pi/180)
        z1 = z*math.cos((store_angle_per_frame[now_frame_num-1])*math.pi/180) - x*math.sin((store_angle_per_frame[now_frame_num-1])*math.pi/180)

        velocity[0] = [x1,0,z1]
        
        if now_frame_num == 1 :
            pre_velocity[0] = velocity[0]
        
        velocity_root[0] = result_motion.getJointVelocityGlobal(0, now_frame_num-1) 
        
        velocity_root[0] = pre_velocity[0] 
        velocity_root[0][1] = velocity[0][1]
        
   
        dot_product = velocity[0][0] *velocity_root[0][0] +velocity[0][1] *velocity[0][1] +velocity[0][2] *velocity_root[0][2]
        len1 = math.sqrt(velocity[0][0] *velocity[0][0] +velocity[0][1] *velocity[0][1] +velocity[0][2] *velocity[0][2])
        len2 = math.sqrt(velocity_root[0][0] *velocity_root[0][0] +velocity[0][1] *velocity[0][1] +velocity_root[0][2] *velocity_root[0][2])
       
        if dot_product/(len1*len2) >= 1.:
            tmp_angle[0] = math.acos( 1.)
        elif dot_product/(len1*len2) <= -1.:
            tmp_angle[0] = math.acos( -1.)
        else :
            tmp_angle[0] = math.acos( dot_product/(len1*len2))
            
        tmp_angle[0] = tmp_angle[0]*180/math.pi
        
        cross_vector[0] = velocity_root[0][2]*velocity[0][0] - velocity_root[0][0]*velocity[0][2]

        if which_style[now_frame_num-1] == 1:
            arrow_color.totalColor = (85,107,47)
        elif which_style[now_frame_num-1] == 2:
            arrow_color.totalColor = (72,61,139)
        elif which_style[now_frame_num-1] == 3:
            arrow_color.totalColor = (139,69,19)
       
        else :
           
            if moving_angle[0] == 'on':
#                arrow_color.totalColor = (75+100,0+100,130+100)
                arrow_color.totalColor = ARROW_MOVING_COLOR
            elif moving_angle[0] == 'off':
#                arrow_color.totalColor =(75,0,130)     
                arrow_color.totalColor = ARROW_FIXED_COLOR     
            
    
            
        
        
        
        
        
        
        if not SIMULATION_ON:
            if now_frame_num > total_frame_num[0]-1:
                call_motionstitch_time(now_frame_num)
           
        
    if SIMULATION_ON:
        if SCREEN_SHOT:
            viewer = ysv.SimpleViewer([150,50,800*2+180,600*2+55])
        else:
            viewer = ysv.SimpleViewer([150,50,800+180,600+55])
    else:
        viewer = ysv.SimpleViewer()
    
    viewer.setCameraTarget(result_motion[0].getJointPositionGlobal(0))
    
    viewer.objectInfoWnd.angle_value_slider.val_slider_handle = val_slider_handle
    viewer.objectInfoWnd.angle_but_cb_handle = angle_but_cb_handle 
    viewer.objectInfoWnd.angle_but_cb = angle_but_cb
    viewer.objectInfoWnd.speed_but_cb = speed_but_cb
    viewer.objectInfoWnd.style_but_cb = style_but_cb
    viewer.motionViewWnd.glWindow.event_handle = event_handle  
    viewer.motionViewWnd.make_motion_seq = make_motion_seq 
    
#===============================================================================
#===============================================================================
#===============================================================================
# # # interactive_control
#===============================================================================
#===============================================================================
#===============================================================================
#def interactive_control():
    class ForceInfo:
        def __init__(self, startFrame, duration, force, targetBody):
            self.startFrame = startFrame    # frame
            self.duration = duration        # sec
            self.force = force              # Newton
            self.targetBody = targetBody

    #===============================================================================
    # load motion
    #===============================================================================
    c_swf_offset = -.025
    
#    c_swf_offset = .01
#    c_swf_offset = .005
    K_stp_pos = 0.
    
#    c5 = .5;    c6 = .01
    c5 = .5;    c6 = .02
#    c5 = .5;    c6 = .05
#    c5 = 1.;    c6 = .05
#    c5 = .0;    c6 = .0
    
    K_stb_vel = .1
    K_stb_pos = .1
    
    OLD_SWING_HEIGHT = False
#    OLD_SWING_HEIGHT = True
    HIGHER_OFFSET = True
#    HIGHER_OFFSET = False

    
    dir = './icmotion_last/'
    
    MULTI_VIEWER = False
    CAMERA_TRACKING = True
    
    # global parameters
#    Kt = 20.;       Dt = 2*(Kt**.5)
#    Ks = 2000.;    Ds = 2*(Ks**.5)
    Kt = 20.;       Dt = 2*(Kt**.5)
    Ks = 2000.;    Ds = 2*(Ks**.5)
    mu = 2.
    
    # constaants
    c_min_contact_vel = 100.
    c_min_contact_time = .7
    c_landing_duration = .2
    c_taking_duration = .3
    c_swf_mid_offset = .07
#    c_swf_mid_offset = .05
#    c_swf_stability = .5
#    c_swf_stability = .0
    c_locking_vel = .05
    
#    K_stb_vel = .1
#    K_stp_pos = 0.
    
#    default_K_swp_pos_sag_faster = .05
#    K_stb_pos = .0

    paramd = {}
    
#    paramd['start'] = {'K_swp_vel_sag':.1, 'K_swp_vel_cor':.4, 'K_swp_pos_sag':1., 'K_swp_pos_cor':0.}
    paramd['start'] = {'K_swp_vel_sag':.1, 'K_swp_vel_cor':.25, 'K_swp_pos_sag':1., 'K_swp_pos_cor':.3, 'K_swp_pos_sag_faster':.05}
    
#    paramd_normal = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.4, 'K_swp_pos_sag':1.5, 'K_swp_pos_cor':0.}
#    paramd_normal = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.25, 'K_swp_pos_sag':1.7, 'K_swp_pos_cor':.3}
    paramd_normal = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':1.8, 'K_swp_pos_cor':.2, 'K_swp_pos_sag_faster':0.}
    paramd['right_left_normal'] = paramd_normal
    paramd['right_right_normal'] = paramd_normal
    paramd['left_left_normal'] = paramd_normal
#    paramd['left_right_normal'] = paramd_normal
#    paramd['left_right_normal'] = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.4, 'K_swp_pos_sag':.5, 'K_swp_pos_cor':0.}
#    paramd['left_right_normal'] = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.25, 'K_swp_pos_sag':.5, 'K_swp_pos_cor':.3}
    paramd['left_right_normal'] = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':.5, 'K_swp_pos_cor':.2, 'K_swp_pos_sag_faster':0.}
    paramd['left_right_normal_long'] = paramd_normal
    paramd['right_left_normal_long'] = paramd_normal 

#    paramd_90_turn = {'K_swp_vel_sag':.1, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':1.2, 'K_swp_pos_cor':0.}
#    paramd_90_turn = {'K_swp_vel_sag':.1, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':1.5, 'K_swp_pos_cor':.4}
    paramd_90_turn = {'K_swp_vel_sag':.1, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':1.5, 'K_swp_pos_cor':.4, 'K_swp_pos_sag_faster':.05}
    paramd['left_right_90_leftturning'] = paramd_90_turn
    paramd['right_left_90_rightturning'] = paramd_90_turn
    
#    paramd_135_turn = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':1.7, 'K_swp_pos_cor':0.}
    paramd_135_turn = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':2., 'K_swp_pos_cor':.4, 'K_swp_pos_sag_faster':.05}
    paramd['right_left_135_rightturning'] = paramd_135_turn
    paramd['left_right_135_leftturning'] = paramd_135_turn
    
#    paramd_180_turn = {'K_swp_vel_sag':.1, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':1.7, 'K_swp_pos_cor':0.}
    paramd_180_turn = {'K_swp_vel_sag':.1, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':2., 'K_swp_pos_cor':.3, 'K_swp_pos_sag_faster':.05}
    paramd['left_right_180_leftturning'] = paramd_180_turn
    paramd['right_left_180_rightturning'] = paramd_180_turn
    
#    paramd_fast = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.4, 'K_swp_pos_sag':1.5, 'K_swp_pos_cor':0.}
    paramd_fast = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.25, 'K_swp_pos_sag':1., 'K_swp_pos_cor':.3, 'K_swp_pos_sag_faster':0.}
    paramd['right_left_fast'] = paramd_fast
    paramd['right_right_fast'] = paramd_fast
    paramd['left_left_fast'] = paramd_fast
    paramd['left_right_fast'] = paramd_fast
#    paramd['left_right_fast'] = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.25, 'K_swp_pos_sag':1., 'K_swp_pos_cor':.3}
#    paramd['left_right_fast'] = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.25, 'K_swp_pos_sag':.5, 'K_swp_pos_cor':.3}

#    paramd_style1 = {'K_swp_vel_sag':.1, 'K_swp_vel_cor':.4, 'K_swp_pos_sag':.5, 'K_swp_pos_cor':.4, 'K_swp_pos_sag_faster':.05}
    paramd_style1 = {'K_swp_vel_sag':.05, 'K_swp_vel_cor':.4, 'K_swp_pos_sag':.5, 'K_swp_pos_cor':.4, 'K_swp_pos_sag_faster':.05}
    paramd['right_left_style1'] = paramd_style1
    paramd['right_right_style1'] = paramd_style1
    paramd['left_left_style1'] = paramd_style1
    paramd['left_right_style1'] = paramd_style1

#    paramd_style2 = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.25, 'K_swp_pos_sag':1.2, 'K_swp_pos_cor':.3}
#    paramd['right_left_style2'] = paramd_style2
#    paramd['right_right_style2'] = paramd_style2
#    paramd['left_left_style2'] = paramd_style2
#    paramd['left_right_style2'] = paramd_style2

#    paramd_style3 = {'K_swp_vel_sag':.0, 'K_swp_vel_cor':.25, 'K_swp_pos_sag':.5, 'K_swp_pos_cor':.3, 'K_swp_pos_sag_faster':.05}
    paramd_style3 = {'K_swp_vel_sag':.1, 'K_swp_vel_cor':.3, 'K_swp_pos_sag':.5, 'K_swp_pos_cor':.3, 'K_swp_pos_sag_faster':.1}
    paramd['right_left_style3'] = paramd_style3
    paramd['right_right_style3'] = paramd_style3
    paramd['left_left_style3'] = paramd_style3
    paramd['left_right_style3'] = paramd_style3
    
    def blend_paramd(paramd1, paramd2, t):
        paramd_blended = {}
        for key in paramd1:
            paramd_blended[key] = (1-t)*paramd1[key] + t*paramd2[key]
        return paramd_blended
            
#    pprint.pprint(blend_paramd(paramd['start'], paramd_normal, .1))

#    right_left_normal = yf.readBvhFile(tempFilePath, 1)
#    right_right_normal = yf.readBvhFile(tempFilePath, 1)    
#    left_left_normal = yf.readBvhFile(tempFilePath, 1)
#    left_right_normal = yf.readBvhFile(tempFilePath, 1)
#    left_right_normal_long = yf.readBvhFile(tempFilePath, 1)
#    right_left_normal_long = yf.readBvhFile(tempFilePath, 1)
#    right_left_fast = yf.readBvhFile(tempFilePath, 1)
#    right_right_fast = yf.readBvhFile(tempFilePath, 1)
#    left_left_fast = yf.readBvhFile(tempFilePath, 1)
#    left_right_fast = yf.readBvhFile(tempFilePath, 1)
#    left_right_45_leftturning = yf.readBvhFile(tempFilePath, 1)
#    left_right_90_leftturning = yf.readBvhFile(tempFilePath, 1)
#    left_right_135_leftturning = yf.readBvhFile(tempFilePath, 1)
#    left_right_180_leftturning = yf.readBvhFile(tempFilePath, 1)
#    right_left_45_rightturning = yf.readBvhFile(tempFilePath, 1)
#    right_left_90_rightturning = yf.readBvhFile(tempFilePath, 1)
#    right_left_135_rightturning = yf.readBvhFile(tempFilePath, 1)
#    right_left_180_rightturning = yf.readBvhFile(tempFilePath, 1)
#    right_left_style1 = yf.readBvhFile(tempFilePath, 1)
#    right_right_style1 = yf.readBvhFile(tempFilePath, 1)
#    left_left_style1 = yf.readBvhFile(tempFilePath, 1)
#    left_right_style1 = yf.readBvhFile(tempFilePath, 1)
#    right_left_style2 = yf.readBvhFile(tempFilePath, 1)
#    right_right_style2 = yf.readBvhFile(tempFilePath, 1)
#    left_left_style2 = yf.readBvhFile(tempFilePath, 1)
#    left_right_style2 = yf.readBvhFile(tempFilePath, 1)
#    right_left_style3 = yf.readBvhFile(tempFilePath, 1)
#    right_right_style3 = yf.readBvhFile(tempFilePath, 1)
#    left_left_style3 = yf.readBvhFile(tempFilePath, 1)
#    left_right_style3 = yf.readBvhFile(tempFilePath, 1)

    frameTime = 1/30.

    #===============================================================================
    # options
    #===============================================================================
    SEGMENT_EDITING =           True
    STANCE_FOOT_STABILIZE =     True
    MATCH_STANCE_LEG =          True
    SWING_FOOT_PLACEMENT =      True
    SWING_FOOT_HEIGHT =         True
    
    SWING_FOOT_ORIENTATION =    False
    
    STANCE_FOOT_PUSH =          True
    STANCE_FOOT_BALANCING =     True
    
    stitch_func = lambda x : 1. - yfg.hermite2nd(x)
    stf_stabilize_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
    match_stl_func = yfg.hermite2nd
    swf_placement_func = yfg.hermite2nd
    swf_height_func = yfg.hermite2nd
    swf_height_sine_func = yfg.sine
#    stf_balancing_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
    stf_balancing_func = yfg.hermite2nd
#    
#    forceInfos = [ForceInfo(70, .4, (100,0,0))]
    forceInfos = []

    #===============================================================================
    # initialize character
    #===============================================================================
    mcfgfile = open(dir + 'mcfg', 'r')
    mcfg = cPickle.load(mcfgfile)
    mcfgfile.close()
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
#    wcfg.useDefaultContactModel = False
    wcfg.useDefaultContactModel = True
    wcfg.lockingVel = c_locking_vel
    stepsPerFrame = 30
    wcfg.timeStep = (frameTime)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    firstPosture = result_motion[0]
    motionModel = cvm.VpMotionModel(vpWorld, firstPosture, mcfg)
    controlModel = cvm.VpControlModel(vpWorld, firstPosture, mcfg)
    
    
    if OBSTACLES:
        # BOXLINE0
        BOX_HEIGHT_NUM = 4
        box0 = [None]*BOX_HEIGHT_NUM
        for i in range(BOX_HEIGHT_NUM):
            box0[i] = cvb.VpBody(vpWorld)
            box0[i].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
            box0[i].setPosition(mm.v3(0,i*BOX_SIZE + BOX_SIZE/2.,0) + BOX0_POS)
            box0[i].setOrientation(mm.rotY(5*mm.RAD*random.random()))
    #    BOX_HEIGHT_NUM = 6
    #    box1 = [None]*BOX_HEIGHT_NUM
    #    for i in range(BOX_HEIGHT_NUM):
    #        box1[i] = cvb.VpBody(vpWorld)
    #        box1[i].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
    #        box1[i].setPosition(mm.v3(0,i*BOX_SIZE + BOX_SIZE/2.,0) + BOX1_POS)
    #        box1[i].setOrientation(mm.rotY(10*mm.RAD*random.random()))
    #    BOX_HEIGHT_NUM = 3
    #    box2 = [None]*BOX_HEIGHT_NUM
    #    for i in range(BOX_HEIGHT_NUM):
    #        box2[i] = cvb.VpBody(vpWorld)
    #        box2[i].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
    #        box2[i].setPosition(mm.v3(0,i*BOX_SIZE + BOX_SIZE/2.,0) + BOX2_POS)
    #        box2[i].setOrientation(mm.rotY(10*mm.RAD*random.random()))
            
        wall0 = [None]*(WALL_HEIGHT_NUM*WALL_HEIGHT_NUM)
        count = 0
        for i in range(WALL_HEIGHT_NUM):
            for j in range(WALL_HEIGHT_NUM):
                wall0[count] = cvb.VpBody(vpWorld)
                wall0[count].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
                wall0[count].setPosition(mm.v3(.05*random.random(), i*BOX_SIZE + BOX_SIZE/2., j*BOX_SIZE) + WALL0_POS)
#                wall0[count].setPosition(mm.v3(j*BOX_SIZE, i*BOX_SIZE + BOX_SIZE/2., .05*random.random()) + WALL0_POS)
                wall0[count].setOrientation(mm.I_SO3())
                count += 1
        
        #box set
       
        box01 = [None]*1
        box01[0] = cvb.VpBody(vpWorld)
        BOX01_POS = (-1.6+-1+x_axis[0],0,-2.6+z_axis[0])
        box01[0].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
        box01[0].setPosition(mm.v3(0,0*BOX_SIZE + BOX_SIZE/2.,0) + BOX01_POS)
        box01[0].setOrientation(mm.rotY(5*mm.RAD*random.random()))
       
        box02 = [None]*1
        box02[0] = cvb.VpBody(vpWorld)
        BOX02_POS = (-1.0-1,0,-3.0)
        box02[0].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
        box02[0].setPosition(mm.v3(0,0*BOX_SIZE + BOX_SIZE/2.,0) + BOX02_POS)
        box02[0].setOrientation(mm.rotY(5*mm.RAD*random.random()))
        
       
        box03 = [None]*1
        box03[0] = cvb.VpBody(vpWorld)
        BOX03_POS = (-1.1-1,0,-2.3) 
        box03[0].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
        box03[0].setPosition(mm.v3(0,0*BOX_SIZE + BOX_SIZE/2.,0) +BOX03_POS)
        box03[0].setOrientation(mm.rotY(5*mm.RAD*random.random()))
          
        
        box04 = [None]*1
        box04[0] = cvb.VpBody(vpWorld)
        BOX04_POS = (-0.9-1,0,-2.6)
        box04[0].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
        box04[0].setPosition(mm.v3(0,1*BOX_SIZE + BOX_SIZE/2.,0) +BOX04_POS)
        box04[0].setOrientation(mm.rotY(5*mm.RAD*random.random()))
       
        
        box05 = [None]*1
        box05[0] = cvb.VpBody(vpWorld)
        BOX05_POS = (-1.5-1,0,-2.5)
        box05[0].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
        box05[0].setPosition(mm.v3(0,1*BOX_SIZE + BOX_SIZE/2.,0) + BOX05_POS)
        box05[0].setOrientation(mm.rotY(5*mm.RAD*random.random()))
       
        box06 = [None]*1
        box06[0] = cvb.VpBody(vpWorld)
        BOX06_POS = (-1.3-1,0,-3.1)
        box06[0].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
        box06[0].setPosition(mm.v3(0,1*BOX_SIZE + BOX_SIZE/2.,0) + BOX06_POS)
        box06[0].setOrientation(mm.rotY(5*mm.RAD*random.random()))
            
    
        box07 = [None]*1
        box07[0] = cvb.VpBody(vpWorld)
        BOX07_POS = (-1.3-1,0,-2.5)
        box07[0].addBoxGeom((BOX_SIZE,BOX_SIZE,BOX_SIZE), BOX_DENSITY)
        box07[0].setPosition(mm.v3(0,2*BOX_SIZE + BOX_SIZE/2.,0) + BOX07_POS)
        box07[0].setOrientation(mm.rotY(5*mm.RAD*random.random()))
    
    
    
    
    
    vpWorld.initialize()
#    print controlModel
    
    controlModel.initializeHybridDynamics()
    
    #===============================================================================
    # load segment info
    #===============================================================================
    skeleton = firstPosture.skeleton
    
    def splitGroup(group_motion):
#        hRef = .2; vRef = .4
        hRef = .2; vRef = .3
        jumpThreshold = 25; jumpBias = 1.
        stopThreshold = 15; stopBias = 0.
        
        lc = yma.getElementContactStates(group_motion, 'LeftFoot', hRef, vRef)
        rc = yma.getElementContactStates(group_motion, 'RightFoot', hRef, vRef)
#        print lc
#        print rc
        intervals, states = yba.getBipedGaitIntervals2(lc, rc, jumpThreshold, jumpBias, stopThreshold, stopBias)
        segments = yma.splitMotionIntoSegments(group_motion, intervals)    
        
        return intervals, states, segments
                
    firstIntervals, firstStates, firstSegments = splitGroup(result_motion)
    firstSegment = firstSegments[0]
    
    
    motion_seg_orig = ym.JointMotion()
    motion_seg_orig += firstSegment
    motion_seg = ym.JointMotion()
    motion_seg += firstSegment
    motion_stitch = ym.JointMotion()
    motion_stitch += firstSegment

    motion_stf_stabilize = ym.JointMotion()
    motion_match_stl = ym.JointMotion()
    motion_swf_placement = ym.JointMotion()
    motion_swf_height = ym.JointMotion()
    motion_swf_orientation = ym.JointMotion()
    motion_stf_balancing = ym.JointMotion()
    motion_stf_push = ym.JointMotion()
    motion_control = ym.JointMotion()

    motion_debug1 = ym.JointMotion()
    motion_debug2 = ym.JointMotion()
    motion_debug3 = ym.JointMotion()
    
    P = ym.JointMotion()
    P_hat = ym.JointMotion()
    M_tc = ym.JointMotion()
    M_hat_tc_1 = ym.JointMotion()
    
    #===============================================================================
    # loop variable
    #===============================================================================
    segments_queue = deque(firstSegments)
    intervals_queue = deque(firstIntervals)
    states_queue = deque(firstStates)
    paramd_queue = deque([paramd['start']]*len(firstSegments))
    
    extended = [False]
    prev_R_swp = [None]
    stl_y_limit_num = [0]
    stl_xz_limit_num = [0]
    avg_dCM = [mm.O_Vec3()]

    step_length_cur = [0.]

    step_length_tar = [0.]
    step_axis = [mm.O_Vec3()]
            
    #===============================================================================
    # information
    #===============================================================================
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [mu]*len(bodyIDsToCheck)
    
    bodyMasses = controlModel.getBodyMasses()
    totalMass = controlModel.getTotalMass()

    lID = controlModel.name2id('LeftFoot');      rID = controlModel.name2id('RightFoot')
    lHip = skeleton.getJointIndex('LeftUpLeg');  rHip = skeleton.getJointIndex('RightUpLeg')
    lKnee = skeleton.getJointIndex('LeftLeg');   rKnee = skeleton.getJointIndex('RightLeg')
    lFoot = skeleton.getJointIndex('LeftFoot');  rFoot = skeleton.getJointIndex('RightFoot')
    spine = skeleton.getJointIndex('Spine')
    
    uppers = [skeleton.getJointIndex(name) for name in ['Hips', 'Spine', 'Spine1', 'LeftArm', 'LeftForeArm', 'RightArm', 'RightForeArm']]
    upperMass = sum([bodyMasses[i] for i in uppers])
    lLegs = [skeleton.getJointIndex(name) for name in ['LeftUpLeg', 'LeftLeg', 'LeftFoot']]
    rLegs = [skeleton.getJointIndex(name) for name in ['RightUpLeg', 'RightLeg', 'RightFoot']]
    allJoints = set(range(skeleton.getJointNum()))
    
    halfFootHeight = controlModel.getBodyShape(lFoot)[1] / 2.
    
#    for fi in forceInfos:
#        fi.targetBody = spine
        
    #===============================================================================
    # rendering
    #===============================================================================
    rd_CM = [None]; rd_CP = [None]; rd_CMP = [None]
    rd_forces = [None]; rd_force_points = [None]
    rd_torques = []; rd_joint_positions = []
    
    rd_point1 = [None]
    rd_point2 = [None]
    rd_vec1 = [None];   rd_vecori1 = [None] 
    rd_vec2 = [None];   rd_vecori2 = [None]
    rd_frame1 = [None]
    rd_frame2 = [None]
    
#    MOTION_COLOR = (192,192,192)
#    CHARACTER_COLOR = (102,102,153)
    MOTION_COLOR = (213,111,162)
    CHARACTER_COLOR = (20,166,188)
    
    
#    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, MOTION_COLOR, polygonStyle))
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, polygonStyle))
    
    
#    
#    viewer.doc.addObject('motion_ori', motion_ori)
#    viewer.doc.addRenderer('motion_ori', yr.JointMotionRenderer(motion_ori, (0,100,255), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_seg_orig', yr.JointMotionRenderer(motion_seg_orig, (0,100,255), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_seg', yr.JointMotionRenderer(motion_seg, (0,150,255), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_stitch', yr.JointMotionRenderer(motion_stitch, (0,255,200), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_stf_stabilize', yr.JointMotionRenderer(motion_stf_stabilize, (255,0,0), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_match_stl', yr.JointMotionRenderer(motion_match_stl, (255,200,0), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_swf_placement', yr.JointMotionRenderer(motion_swf_placement, (255,100,255), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_swf_height', yr.JointMotionRenderer(motion_swf_height, (50,255,255), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_swf_orientation', yr.JointMotionRenderer(motion_swf_orientation, (255,100,0), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_stf_push', yr.JointMotionRenderer(motion_stf_push, (50,255,200), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_stf_balancing', yr.JointMotionRenderer(motion_stf_balancing, (255,100,255), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_control', yr.JointMotionRenderer(motion_control, (255,0,0), yr.LINK_BONE))
#
#    viewer.doc.addRenderer('motion_debug1', yr.JointMotionRenderer(motion_debug1, (0,255,0), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_debug2', yr.JointMotionRenderer(motion_debug2, (255,0,255), yr.LINK_BONE))
#    viewer.doc.addRenderer('motion_debug3', yr.JointMotionRenderer(motion_debug3, (255,255,0), yr.LINK_BONE))
#
#    viewer.doc.addRenderer('M_tc', yr.JointMotionRenderer(M_tc, (255,255,0), yr.LINK_BONE))
#    viewer.doc.addRenderer('P_hat', yr.JointMotionRenderer(P_hat, (255,255,0), yr.LINK_BONE))
#    viewer.doc.addRenderer('P', yr.JointMotionRenderer(P, (255,255,0), yr.LINK_BONE))
#    viewer.doc.addRenderer('M_hat_tc_1', yr.JointMotionRenderer(M_hat_tc_1, (255,255,0), yr.LINK_BONE))
#
#    viewer.doc.addRenderer('rd_CM', yr.PointsRenderer(rd_CM, (255,255,0)))
#    viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (255,0,0)))
#    viewer.doc.addRenderer('rd_CMP', yr.PointsRenderer(rd_CMP, (0,255,0)))
   #force_arrow
    arrow_force = yr.ForcesRenderer(rd_forces, rd_force_points, (255,0,0), ratio= 0.01, fromPoint=False, lineWidth=.03)
    viewer.doc.addRenderer('forces',arrow_force )
#    viewer.doc.addRenderer('torques', yr.VectorsRenderer(rd_torques, rd_joint_positions, (255,0,0)))
#
#    viewer.doc.addRenderer('rd_point1', yr.PointsRenderer(rd_point1, (0,255,0)))
#    viewer.doc.addRenderer('rd_point2', yr.PointsRenderer(rd_point2, (255,0,0)))
#    viewer.doc.addRenderer('rd_vec1', yr.VectorsRenderer(rd_vec1, rd_vecori1, (255,0,0)))
#    viewer.doc.addRenderer('rd_vec2', yr.VectorsRenderer(rd_vec2, rd_vecori2, (0,255,0)))
#    viewer.doc.addRenderer('rd_frame1', yr.FramesRenderer(rd_frame1, (0,200,200)))
#    viewer.doc.addRenderer('rd_frame2', yr.FramesRenderer(rd_frame2, (200,200,0)))
#    viewer.setMaxFrame(len(motion_ori)-1)

    viewer.setMaxFrame(10000)
        
    if CAMERA_TRACKING:
        if MULTI_VIEWER:
            cameraTargets1 = [None] * (viewer.getMaxFrame()+1)
            cameraTargets2 = [None] * (viewer.getMaxFrame()+1)
        else:
            cameraTargets = [None] * (viewer.getMaxFrame()+1)
            
    pt = [0.]
    def postFrameCallback_Always(frame):
        if frame==1: pt[0] = time.time()
        if frame==31: print 'elapsed time for 30 frames:', time.time()-pt[0]
        if CAMERA_TRACKING:
            if MULTI_VIEWER:
                if cameraTargets1[frame]==None:
                    cameraTargets1[frame] = motionModel.getBodyPositionGlobal(0)
#                    cameraTargets1[frame] = motion_ori[frame].getJointPositionGlobal(0)
                viewer.setCameraTarget1(cameraTargets1[frame])
                
                if cameraTargets2[frame]==None:
                    cameraTargets2[frame] = controlModel.getJointPositionGlobal(0)
                viewer.setCameraTarget2(cameraTargets2[frame])
                
            else:
                if cameraTargets[frame]==None:
                    cameraTargets[frame] = controlModel.getJointPositionGlobal(0)
                viewer.setCameraTarget(cameraTargets[frame])
        if plot!=None:
            plot.updateVline(frame)

    if SIMULATION_ON:
        viewer.setPostFrameCallback_Always(postFrameCallback_Always)

    plot = None
#    plot = ymp.InteractivePlot()
    if plot!=None:
        plot.setXlimit(0, len(motion_ori))
        plot.setYlimit(0., 1.)
        plot.addDataSet('zero')
        plot.addDataSet('diff')
        plot.addDataSet('debug1')
        plot.addDataSet('debug2')

    def viewer_onClose(data):
        if plot!=None:
            plot.close()
        viewer.onClose(data)
    viewer.callback(viewer_onClose)
         
    def simulateCallback(frame):
        # seginfo
        curState = states_queue[0]
        curInterval = intervals_queue[0]
        curParamd = paramd_queue[0]
        groundHeight = 0.

        K_swp_vel_sag = curParamd['K_swp_vel_sag']; K_swp_vel_cor = curParamd['K_swp_vel_cor']
        K_swp_pos_sag = curParamd['K_swp_pos_sag']; K_swp_pos_cor = curParamd['K_swp_pos_cor']
        
        if 'K_swp_pos_sag_faster' in curParamd:
            K_swp_pos_sag_faster = curParamd['K_swp_pos_sag_faster']
        else:
            K_swp_pos_sag_faster = default_K_swp_pos_sag_faster
            
#        pprint.pprint(curParamd)

        stanceLegs = []; swingLegs = []; stanceFoots = []; swingFoots = []; swingKnees = []
        if curState==yba.GaitState.LSWING:   stanceLegs = [rHip]; stanceFoots = [rFoot]; swingLegs = [lHip]; swingFoots = [lFoot]; swingKnees = [lKnee]
        elif curState==yba.GaitState.RSWING: stanceLegs = [lHip]; stanceFoots = [lFoot]; swingLegs = [rHip]; swingFoots = [rFoot]; swingKnees = [rKnee]
        elif curState==yba.GaitState.STOP:   stanceLegs = [rHip, lHip]; stanceFoots = [rFoot, lFoot]
        elif curState==yba.GaitState.JUMP:   swingLegs = [rHip, lHip]; swingFoots = [rFoot, lFoot]
        
        prev_frame = frame-1 if frame>0 else 0
#        prev_frame = frame

        # information
        dCM_tar = motion_seg.getJointVelocityGlobal(0, prev_frame)
        CM_tar = motion_seg.getJointPositionGlobal(0, prev_frame)
        stf_tar = motion_seg.getJointPositionGlobal(stanceFoots[0], prev_frame)
        CMr_tar = CM_tar - stf_tar
            
        dCM = avg_dCM[0]
        CM = controlModel.getJointPositionGlobal(0)
        CMreal = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, totalMass)
        stf = controlModel.getJointPositionGlobal(stanceFoots[0])
        CMr = CM - stf
        
        diff_dCM = mm.projectionOnPlane(dCM-dCM_tar, (1,0,0), (0,0,1))
        diff_dCM_axis = np.cross((0,1,0), diff_dCM)
        rd_vec1[0] = diff_dCM; rd_vecori1[0] = CM_tar
        
        diff_CMr = mm.projectionOnPlane(CMr-CMr_tar, (1,0,0), (0,0,1))
#        rd_vec1[0] = diff_CMr; rd_vecori1[0] = stf_tar
        diff_CMr_axis = np.cross((0,1,0), diff_CMr)
        
#        direction = mm.normalize2(mm.projectionOnPlane(motion_seg.getJointOrientationGlobal(0, frame).T[2], (1,0,0), (0,0,1)))
#        direction = mm.normalize2(mm.projectionOnPlane(controlModel.getJointOrientationGlobal(0).T[2], (1,0,0), (0,0,1)))
        direction = mm.normalize2(mm.projectionOnPlane(dCM_tar, (1,0,0), (0,0,1)))
#        direction = mm.normalize2(mm.projectionOnPlane(dCM, (1,0,0), (0,0,1)))
        directionAxis = np.cross((0,1,0), direction)
#        rd_vec1[0] = direction; rd_vecori1[0] = CM
        
        diff_dCM_sag, diff_dCM_cor = mm.projectionOnVector2(diff_dCM, direction)
#        rd_vec1[0] = diff_dCM_sag; rd_vecori1[0] = CM_tar
        diff_dCM_sag_axis = np.cross((0,1,0), diff_dCM_sag)
        diff_dCM_cor_axis = np.cross((0,1,0), diff_dCM_cor)
            
        diff_CMr_sag, diff_CMr_cor = mm.projectionOnVector2(diff_CMr, direction)
        diff_CMr_sag_axis = np.cross((0,1,0), diff_CMr_sag)
        diff_CMr_cor_axis = np.cross((0,1,0), diff_CMr_cor)
            
        t = (frame-curInterval[0])/float(curInterval[1]-curInterval[0])
        t_raw = t
        if t>1.: t=1.
        
        p_root = motion_stitch[frame].getJointPositionGlobal(0)
        R_root = motion_stitch[frame].getJointOrientationGlobal(0)

        motion_seg_orig.goToFrame(frame)
        motion_seg.goToFrame(frame)
        motion_stitch.goToFrame(frame)
        
#        motion_debug1.append(motion_stitch[frame].copy())
#        motion_debug1.goToFrame(frame)
#        motion_debug2.append(motion_stitch[frame].copy())
#        motion_debug2.goToFrame(frame)
#        motion_debug3.append(motion_stitch[frame].copy())
#        motion_debug3.goToFrame(frame)
        
        # paper implementation
        M_tc.append(motion_stitch[prev_frame])
        M_tc.goToFrame(frame)
        P_hat.append(M_tc[frame].copy())
        P_hat.goToFrame(frame)
        
        p_temp = ym.JointPosture(skeleton)
        p_temp.rootPos = controlModel.getJointPositionGlobal(0)
        p_temp.setJointOrientationsLocal(controlModel.getJointOrientationsLocal())
        P.append(p_temp)
        P.goToFrame(frame)
        
        # stance foot stabilize
        motion_stf_stabilize.append(motion_stitch[frame].copy())
        motion_stf_stabilize.goToFrame(frame)
        if STANCE_FOOT_STABILIZE:
            for stanceFoot in stanceFoots:
                R_target_foot = motion_seg[frame].getJointOrientationGlobal(stanceFoot)
                R_current_foot = motion_stf_stabilize[frame].getJointOrientationGlobal(stanceFoot)
                motion_stf_stabilize[frame].setJointOrientationGlobal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))
#                R_target_foot = motion_seg[frame].getJointOrientationLocal(stanceFoot)
#                R_current_foot = motion_stf_stabilize[frame].getJointOrientationLocal(stanceFoot)
#                motion_stf_stabilize[frame].setJointOrientationLocal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))

        # match stance leg 
        motion_match_stl.append(motion_stf_stabilize[frame].copy())
        motion_match_stl.goToFrame(frame)
        if MATCH_STANCE_LEG:
            if curState!=yba.GaitState.STOP:
                for i in range(len(stanceLegs)):
                    stanceLeg = stanceLegs[i]
                    stanceFoot = stanceFoots[i]
                    
#                    # motion stance leg -> character stance leg as time goes
                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
                    R_character = controlModel.getJointOrientationGlobal(stanceLeg)
                    motion_match_stl[frame].setJointOrientationGlobal(stanceLeg, cm.slerp(R_motion, R_character, match_stl_func(t)))

#                    t_y = match_stl_func_y(t)
#                    t_xz = match_stl_func(t)
#                    
#                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
#                    R_character = controlModel.getJointOrientationGlobal(stanceLeg)
#                    R = np.dot(R_character, R_motion.T)
#                    R_y, R_xz = mm.projectRotation((0,1,0), R)
#                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_xz, t_xz))
#                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_y, t_y))

        # swing foot placement
        motion_swf_placement.append(motion_match_stl[frame].copy())
        motion_swf_placement.goToFrame(frame)
        if SWING_FOOT_PLACEMENT:
            t_swing_foot_placement = swf_placement_func(t);
            
            if extended[0]:
                R_swp_sag = prev_R_swp[0][0]
                R_swp_cor = prev_R_swp[0][1]
            else:
                R_swp_sag = mm.I_SO3(); R_swp_cor = mm.I_SO3()
                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * K_swp_vel_sag * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_dCM_cor_axis * K_swp_vel_cor * -t_swing_foot_placement))
                if np.dot(direction, diff_CMr_sag) < 0:
                    R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * K_swp_pos_sag * -t_swing_foot_placement))
                else:
                    R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * K_swp_pos_sag_faster * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_CMr_cor_axis * K_swp_pos_cor * -t_swing_foot_placement))

            for i in range(len(swingLegs)):
                swingLeg = swingLegs[i]
                swingFoot = swingFoots[i] 
                
                # save swing foot global orientation
#                R_swf = motion_swf_placement[frame].getJointOrientationGlobal(swingFoot)
                
                # rotate swing leg
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_sag)
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_cor)
                
                # restore swing foot global orientation
#                motion_swf_placement[frame].setJointOrientationGlobal(swingFoot, R_swf)
                
                prev_R_swp[0] = (R_swp_sag, R_swp_cor)

        # swing foot height
        motion_swf_height.append(motion_swf_placement[frame].copy())
        motion_swf_height.goToFrame(frame)
        if SWING_FOOT_HEIGHT:
            for swingFoot in swingFoots:
                stanceFoot = stanceFoots[0]

                # save foot global orientation
                R_foot = motion_swf_height[frame].getJointOrientationGlobal(swingFoot)
                R_stance_foot = motion_swf_height[frame].getJointOrientationGlobal(stanceFoot)

                if OLD_SWING_HEIGHT:
                    height_tar = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]
                else:
                    height_tar = motion_swf_height[prev_frame].getJointPositionGlobal(swingFoot)[1] - groundHeight
                    d_height_tar = motion_swf_height.getJointVelocityGlobal(swingFoot, prev_frame)[1]
#                    height_tar += c_swf_mid_offset * swf_height_sine_func(t)
#                motion_debug1[frame] = motion_swf_height[frame].copy()

                # rotate
                motion_swf_height[frame].rotateByTarget(controlModel.getJointOrientationGlobal(0))
#                motion_debug2[frame] = motion_swf_height[frame].copy()
#                motion_debug2[frame].translateByTarget(controlModel.getJointPositionGlobal(0))

                if OLD_SWING_HEIGHT:
                    height_cur = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]
                else:
                    height_cur = controlModel.getJointPositionGlobal(swingFoot)[1] - halfFootHeight - c_swf_offset
                    d_height_cur = controlModel.getJointVelocityGlobal(swingFoot)[1]

                if OLD_SWING_HEIGHT:
                    offset_height = (height_tar - height_cur) * swf_height_func(t) * c5
                else:
                    offset_height = ((height_tar - height_cur) * c5
                                     + (d_height_tar - d_height_cur) * c6) * swf_height_func(t)

                offset_sine = c_swf_mid_offset * swf_height_sine_func(t)
#                offset_sine = 0.
                
                offset = 0.
                offset += offset_height
                offset += offset_sine

                if offset > 0.:
                    newPosition =  motion_swf_height[frame].getJointPositionGlobal(swingFoot)
                    newPosition[1] += offset
                    aik.ik_analytic(motion_swf_height[frame], swingFoot, newPosition)
                else:
                    if HIGHER_OFFSET:
                        newPosition =  motion_swf_height[frame].getJointPositionGlobal(stanceFoot)
                        newPosition[1] -= offset
                        aik.ik_analytic(motion_swf_height[frame], stanceFoot, newPosition)

                # return
#                motion_debug3[frame] = motion_swf_height[frame].copy()
#                motion_debug3[frame].translateByTarget(controlModel.getJointPositionGlobal(0))
                motion_swf_height[frame].rotateByTarget(R_root)
                
                # restore foot global orientation
                motion_swf_height[frame].setJointOrientationGlobal(swingFoot, R_foot)
                motion_swf_height[frame].setJointOrientationGlobal(stanceFoot, R_stance_foot)

                if plot!=None:
                    plot.addDataPoint('debug1', frame, offset_height)
                    plot.addDataPoint('debug2', frame, height_tar - height_cur)
#                    plot.addDataPoint('diff', frame, diff)


        # stance foot push                
        motion_stf_push.append(motion_swf_height[frame].copy())
        motion_stf_push.goToFrame(frame)
        if STANCE_FOOT_PUSH:
            for swingFoot in swingFoots:
#                max_t = (maxStfPushFrame)/float(curInterval[1]-curInterval[0])
#                stf_push_func = yfg.concatenate([yfg.sine, yfg.zero], [max_t*2])
                stf_push_func = yfg.concatenate([yfg.sine, yfg.zero], [c_taking_duration*2])
                
                R_swp_sag = mm.I_SO3()
#                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * K_stp_vel * -stf_push_func(t)))
                
#                if step_length_cur[0] < step_length_tar[0]:
#                    ratio = step_length_cur[0] / step_length_tar[0]
#                    R_max = maxmaxStfPushFrame

#                R_swp_sag = np.dot(R_swp_sag, mm.exp((step_length_tar[0] - step_length_cur[0])*step_axis[0] * K_stp_pos * -stf_push_func(t)))

                R_swp_sag = np.dot(R_swp_sag, mm.exp((1)*step_axis[0] * K_stp_pos * -stf_push_func(t)))
                    
                motion_stf_push[frame].mulJointOrientationGlobal(swingFoot, R_swp_sag)
                
        # stance foot balancing 
        motion_stf_balancing.append(motion_stf_push[frame].copy())
        motion_stf_balancing.goToFrame(frame)
        if STANCE_FOOT_BALANCING:
            R_stb = mm.exp(diff_dCM_axis * K_stb_vel * stf_balancing_func(t))
            R_stb = np.dot(R_stb, mm.exp(diff_CMr_axis * K_stb_pos * stf_balancing_func(t)))
            for stanceFoot in stanceFoots:
                if frame < 5: continue
                motion_stf_balancing[frame].mulJointOrientationGlobal(stanceFoot, R_stb)
                    
        # control trajectory
        motion_control.append(motion_stf_balancing[frame].copy())
        motion_control.goToFrame(frame)
        
        #=======================================================================
        # tracking with inverse dynamics
        #=======================================================================
        th_r = motion_control.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion_control.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion_control.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)

        #=======================================================================
        # simulation
        #=======================================================================
        avg_dCM[0] = mm.v3(0.,0.,0.)
        
        # external force rendering info
        del rd_forces[:]; del rd_force_points[:]
        for fi in forceInfos:
            if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/frameTime):
                rd_forces.append(fi.force)
                rd_force_points.append(controlModel.getBodyPositionGlobal(fi.targetBody) + -mm.normalize2(fi.force)*.2)
            elif frame >= fi.startFrame + fi.duration*(1/frameTime):
                del forceInfos[:]
                    
        for i in range(stepsPerFrame):
            
            bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)
            
            # apply external force
            for fi in forceInfos:
                if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/frameTime):
                    controlModel.applyBodyForceGlobal(fi.targetBody, fi.force)
            
            controlModel.setDOFAccelerations(ddth_des)
            controlModel.solveHybridDynamics()
            
            vpWorld.step()

            avg_dCM[0] += controlModel.getJointVelocityGlobal(0)

        avg_dCM[0] /= stepsPerFrame
        
        #=======================================================================
        # segment editing
        #=======================================================================
        lastFrame = False
        
        if SEGMENT_EDITING:
            if curState==yba.GaitState.STOP:
                if frame == len(motion_seg)-1:
                    lastFrame = True
                    
            elif (curState==yba.GaitState.LSWING or curState==yba.GaitState.RSWING) and t>c_min_contact_time:
                swingID = lID if curState==yba.GaitState.LSWING else rID

                contact = False
                if swingID in bodyIDs:
                    minContactVel = 1000.
                    for i in range(len(bodyIDs)):
                        if bodyIDs[i]==swingID:
#                            vel = controlModel.getBodyVelocityGlobal(swingID, contactPositionLocals[i])
                            vel = controlModel.getBodyVelocityGlobal(controlModel.id2index(swingID), contactPositionLocals[i])
                            vel[1] = 0
                            contactVel = mm.length(vel)
                            if contactVel < minContactVel: minContactVel = contactVel 
                    if minContactVel < c_min_contact_vel: contact = True
                
                extended[0] = False
                
                if contact:
#                    print frame, 'foot touch'
                    lastFrame = True
#                    acc_offset[0] += frame - curInterval[1]
                    
                elif frame == len(motion_seg)-1:
#                    print frame, 'extend frame', frame+1
                    
                    preserveJoints = []
#                    preserveJoints = [lFoot, rFoot]
#                    preserveJoints = [lFoot, rFoot, lKnee, rKnee]
#                    preserveJoints = [lFoot, rFoot, lKnee, rKnee, lUpLeg, rUpLeg]
                    stanceKnees = [rKnee] if curState==yba.GaitState.LSWING else [lKnee]   
                    preserveJoints = [stanceFoots[0], stanceKnees[0], stanceLegs[0]]
   
                    diff = 3
                    motion_seg_orig.extend([motion_seg_orig[-1]])
                    motion_seg.extend(ymt.extendByIntegration_root(motion_seg, 1, diff))
                    
                    motion_stitch.extend(ymt.extendByIntegration_constant(motion_stitch, 1, preserveJoints, diff))

#                    # extend for swing foot ground speed matching & swing foot height lower
##                    extendedPostures = ymt.extendByIntegration(motion_stitch, 1, preserveJoints, diff)
##                    extendedPostures = [motion_stitch[-1]] 
##
#                    extendFrameNum = frame - curInterval[1] + 1
#                    k = 1.-extendFrameNum/5.
#                    if k<0.: k=0.
#                    extendedPostures = ymt.extendByIntegrationAttenuation(motion_stitch, 1, preserveJoints, diff, k)
#
##                    if len(swingFoots)>0 and np.inner(dCM_tar, dCM)>0.:
##                        print frame, 'speed matching'
##                        R_swf = motion_stitch[-1].getJointOrientationGlobal(swingFoots[0])
##                        
##                        p_swf = motion_stitch[-1].getJointPositionGlobal(swingFoots[0])
##                        v_swf = motion_stitch.getJointVelocityGlobal(swingFoots[0], frame-diff, frame)
##                        a_swf = motion_stitch.getJointAccelerationGlobal(swingFoots[0], frame-diff, frame)
##                        p_swf += v_swf * (frameTime) + a_swf * (frameTime)*(frameTime)
##                        aik.ik_analytic(extendedPostures[0], swingFoots[0], p_swf)
##                        
##                        extendedPostures[0].setJointOrientationGlobal(swingFoots[0], R_swf)
#
#                    motion_stitch.extend(extendedPostures)

                    result_motion.append(result_motion[-1])
                    
                    extended[0] = True
        else:
            if frame == len(motion_seg)-1: lastFrame = True
                    
        if lastFrame:
            if True:
#            if segIndex < len(segments)-1:
#                print '%d (%d): end of %dth seg (%s, %s)'%(frame, frame-curInterval[1],segIndex, yba.GaitState.text[curState], curInterval)
                if plot!=None: plot.addDataPoint('diff', frame, (frame-curInterval[1])*.01)
                
                if len(stanceFoots)>0 and len(swingFoots)>0:
#                    step_cur = controlModel.getJointPositionGlobal(swingFoots[0]) - controlModel.getJointPositionGlobal(stanceFoots[0])
#                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(swingFoots[0]) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                    step_cur = controlModel.getJointPositionGlobal(0) - controlModel.getJointPositionGlobal(stanceFoots[0])
                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(0) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                    
                    step_cur = mm.projectionOnPlane(step_cur, (1,0,0), (0,0,1))
                    step_tar = mm.projectionOnPlane(step_tar, (1,0,0), (0,0,1))
                    
                    step_cur_sag, step_cur_cor = mm.projectionOnVector2(step_cur, direction)
                    step_tar_sag, step_tar_cor = mm.projectionOnVector2(step_tar, direction)
                    
                    step_length_tar[0] = mm.length(step_tar_sag)
                    if np.inner(step_tar_sag, step_cur_sag) > 0:
                        step_length_cur[0] = mm.length(step_cur_sag)
                    else:
                        step_length_cur[0] = -mm.length(step_cur_sag)
                    
                    step_axis[0] = directionAxis
                    
#                    rd_vec1[0] = step_tar_sag
#                    rd_vecori1[0] = motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
#                    rd_vec2[0] = step_cur_sag
#                    rd_vecori2[0] = controlModel.getJointPositionGlobal(stanceFoots[0])

                # get next segment group
                segments_queue.popleft()
                intervals_queue.popleft()
                states_queue.popleft()
                paramd_queue.popleft()
                
#                print 
#                print 'lastFrame', frame
                if len(segments_queue)==0:
                    next_group, motion_info = call_motionstitch_time(frame)
                    
                    next_intervals, next_states, next_segments = splitGroup(next_group)
#                    print 'next states', [yba.GaitState.text[ns] for ns in next_states]
#                    print 'next intervals', next_intervals
#                    print next_segments
                    segments_queue.extend(next_segments)
                    intervals_queue.extend(next_intervals)
                    states_queue.extend(next_states)

#                    motion_info = [[smooth_blended_file1, smooth_blended_file2, start_t, end_t], [blended_file1, blended_file2, t]]
#                                                                                              or [stitched_file]]
                    print frame, 'motion_info', motion_info
                    
                    if len(motion_info[1])==1:
                        stitched = motion_info[1][0]
                        target_paramd = paramd[stitched]
                    elif len(motion_info[1])==3:
                        blended1 = motion_info[1][0]
                        blended2 = motion_info[1][1]
                        blend_t = motion_info[1][2]
                        target_paramd = blend_paramd(paramd[blended1], paramd[blended2], blend_t)
                    else:
#                        print 'ERROR'
                        target_paramd = paramd_normal
                    
                    if len(next_segments)==1:
                        next_paramds = [target_paramd]
                    else:
                        next_paramds = [target_paramd]*len(next_segments)
                        next_paramds[0] = blend_paramd(curParamd, target_paramd, .5)
#                        print 'cur_param', curParamd
#                        print 'next_param[0]', next_paramds[0]
#                        print 'target_param', target_paramd
                                
                    paramd_queue.extend(next_paramds)

                    
                curSeg = segments_queue[0]
                offset = frame - intervals_queue[0][0]
                intervals_queue[0][0] += offset
                intervals_queue[0][1] += offset
                
#                stl_y_limit_num[0] = 0
#                stl_xz_limit_num[0] = 0
                
                del motion_seg_orig[frame+1:]
                motion_seg_orig.extend(ymb.getAttachedNextMotion(curSeg, motion_seg_orig[-1], False, False))
                
                del motion_seg[frame+1:]
                del motion_stitch[frame+1:]
                transitionLength = len(curSeg)-1

#                motion_seg.extend(ymb.getAttachedNextMotion2(curSeg, motion_seg[-1], True, False))
#                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, motion_control[-1], transitionLength, stitch_func, True, False))
                d = motion_seg[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, d, True, False))
                d = motion_control[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, d, transitionLength, stitch_func, True, False))

            else:
                motion_seg_orig.append(motion_seg_orig[-1])
                motion_seg.append(motion_seg[-1])
                motion_stitch.append(motion_control[-1])
                
                
        # rendering
#        motionModel.update(motion_ori[frame])
        motionModel.update(motion_seg[frame])
        
        
    if SIMULATION_ON:
        viewer.setSimulateCallback(simulateCallback)
        
#===============================================================================
#===============================================================================
#===============================================================================
# # # interactive_control end
#===============================================================================
#===============================================================================
#===============================================================================
    
    viewer.motionViewWnd.glWindow.floor_axis = [None]
    viewer.motionViewWnd.glWindow.floor_axis[0] = [-10,0,-10]
   
#    viewer.record(False)
#    arrow_color = yr.ForcesRenderer(velocity,direction,(255,0,0), 1., .03)
#    arrow_color = yr.WideArrowRenderer(velocity,direction,(75,0,130), 2., .1, polygonStyle=polygonStyle)
    arrow_color = yr.WideArrowRenderer(velocity,direction, ARROW_FIXED_COLOR, 2., .1, polygonStyle=polygonStyle)
    viewer.doc.addRenderer('arrow', arrow_color)
    
#    viewer.doc.addRenderer('result_motion', yr.JointMotionRenderer(right_left_135_rightturning, (255,180,192), lineWidth=5.))
#    viewer.doc.addObject('result_motion',right_left_135_rightturning)

    if SHOW_MOTION:
#        viewer.doc.addRenderer('result_motion', yr.JointMotionRenderer(result_motion, (255,218,185), lineWidth=5., linkStyle=linkStyle))
#        viewer.doc.addObject('result_motion',result_motion)
        viewer.doc.addRenderer('motion_seg_orig', yr.JointMotionRenderer(motion_seg_orig, (255,218,185), lineWidth=5., linkStyle=linkStyle))
        viewer.doc.addObject('motion_seg_orig',motion_seg_orig)

    if OBSTACLES:
    #    viewer.doc.addRenderer('root_frame', yr.FramesRenderer(root_frame, (0,0,0)))
        viewer.doc.addRenderer('box0', yr.VpBodiesRenderer(box0, (255,0,124), polygonStyle))
    #    viewer.doc.addRenderer('box1', yr.VpBodiesRenderer(box1, (255,255,0)))
    #    viewer.doc.addRenderer('box2', yr.VpBodiesRenderer(box2, (53,124,255)))
        viewer.doc.addRenderer('wall0', yr.VpBodiesRenderer(wall0, (158,5,255), polygonStyle))
       
        viewer.doc.addRenderer('box01',yr.VpBodiesRenderer(box01, (255-80,153-80,204-80), polygonStyle))
        viewer.doc.addRenderer('box02', yr.VpBodiesRenderer(box02, (255-80,102-80,0), polygonStyle))
        viewer.doc.addRenderer('box03', yr.VpBodiesRenderer(box03, (204-80,255-80,255-80), polygonStyle))
        viewer.doc.addRenderer('box04', yr.VpBodiesRenderer(box04, (204-80,255-80,204-80), polygonStyle))
        viewer.doc.addRenderer('box05', yr.VpBodiesRenderer(box05, (204-80,153-80,255-80), polygonStyle))
        viewer.doc.addRenderer('box06', yr.VpBodiesRenderer(box06, (255-80,204-80,153-80), polygonStyle))
        viewer.doc.addRenderer('box07', yr.VpBodiesRenderer(box07, (153-80,204-80,255-80), polygonStyle))
        viewer.doc.addObject('result_motion',result_motion) 
    viewer.setMaxFrame(len(result_motion))
    
    if WIREFRAME:
        viewer.motionViewWnd.glWindow.wireframe = True
    
#    viewer.startTimer(1/30.)
    viewer.startTimer((1/30.)*.1)
    viewer.show()
   
    Fl.run()
    
     

#test_seg()
#motion_crop_store_another_bvh1() 

motion_control()

#profileDataFile = './profile_data/profile_motion_control_%s.profile'%datetime.today().strftime('%y%m%d_%H%M%S')
#cProfile.runctx('motion_control()', globals(), locals(), profileDataFile)
#os.system('c:\python25\python.exe ./pprofui.py %s'%profileDataFile)
