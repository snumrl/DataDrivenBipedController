# +-------------------------------------------------------------------------
# | ysControl.py
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

import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm

# th_r[0], th[0] : (Vec3, SO3), th_r[1:], th[1:] : SO3
# dth_r, dth : Vec3
# ddth_r : Vec3
def getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt):
    ddth_des = [None]*len(th_r)
    
#    p_r0 = mm.T2p(th_r[0])
#    p0 = mm.T2p(th[0])
#    v_r0 = dth_r[0][0:3]
#    v0 = dth[0][0:3]
#    a_r0 = ddth_r[0][0:3]
#    th_r0 = mm.T2R(th_r[0])
#    th0 = mm.T2R(th[0])
#    dth_r0 = dth_r[0][3:6]
#    dth0 = dth[0][3:6]
#    ddth_r0 = ddth_r[0][3:6]

    p_r0 = th_r[0][0]
    p0 = th[0][0]
    v_r0 = dth_r[0][0:3]
    v0 = dth[0][0:3]
    a_r0 = ddth_r[0][0:3]
    
    th_r0 = th_r[0][1]
    th0 = th[0][1]
    dth_r0 = dth_r[0][3:6]
    dth0 = dth[0][3:6]
    ddth_r0 = ddth_r[0][3:6]
    
    a_des0 = Kt*(p_r0 - p0) + Dt*(v_r0 - v0) + a_r0
    ddth_des0 = Kt*(mm.logSO3(np.dot(th0.transpose(), th_r0))) + Dt*(dth_r0 - dth0) + ddth_r0
    ddth_des[0] = np.concatenate((a_des0, ddth_des0))
    
    for i in range(1, len(th_r)):
        ddth_des[i] = Kt*(mm.logSO3(np.dot(th[i].transpose(), th_r[i]))) + Dt*(dth_r[i] - dth[i]) + ddth_r[i]

    return ddth_des

# th_r, th : SO3
# dth_r, dth : Vec3
# ddth_r : Vec3
def getDesiredAngAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt):
    ddth_des = [None]*len(th_r) 
    for i in range(len(th_r)):
        ddth_des[i] = Kt*(mm.logSO3(np.dot(th[i].transpose(), th_r[i]))) + Dt*(dth_r[i] - dth[i]) + ddth_r[i]

    return ddth_des

# p_r, p, v_r, v, a_r : Vec3
def getDesiredAcceleration(p_r, p, v_r, v, a_r, Kt, Dt):
    return Kt*(p_r - p) + Dt*(v_r - v) + a_r


if __name__ == '__main__':
    import psyco; psyco.full()
    from fltk import *
    
    import Resource.ysMotionLoader as yf
    import Simulator.ysPhysConfig as ypc
    import Renderer.ysRenderer as yr
    import Renderer.csVpRenderer as cvr
    import Simulator.csVpWorld as cvw
    import Simulator.csVpModel as cvm
    import GUI.ysSimpleViewer as ysv
    import Optimization.csEQP as ceq
    import ArticulatedBody.ysJacobian as yjc
    import Util.ysPythonEx as ype
    import Motion.ysSkeletonEdit as yme
    import ArticulatedBody.ysMomentum as ymt

    
    def test_getDesiredAngAccelerations():
#        motion = yf.readBvhFile('block_3_rotate.bvh', 1)
        motion = yf.readBvhFile('../samples/block_tree_rotate.bvh', 1)
        motion = motion[0:]
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .8
        for i in range(motion[0].skeleton.getElementNum()):
            mcfg.addNode(motion[0].skeleton.getElementName(i))
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = -1.
        wcfg.gravity = (0,0,0)
        stepsPerFrame = 30
        wcfg.timeStep = (1/30.)/stepsPerFrame
        
        vpWorld = cvw.VpWorld(wcfg)
        motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
        controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
        vpWorld.initialize()
        controlModel.initializeHybridDynamics()
        
        controlModel.fixBody(0)
    
    
        p = []
        ddth_des = []
    
        viewer = ysv.SimpleViewer()
    #    viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (255,240,255), yr.POLYGON_LINE))
        viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL))
        
        viewer.doc.addRenderer('ddth_des', yr.VectorsRenderer(ddth_des, p, (255,0,0)))
        
        def simulateCallback(frame):
            th_r = motion.getInternalJointOrientationsLocal(frame)
            th = controlModel.getInternalJointOrientationsLocal()
            dth_r = motion.getInternalJointAngVelocitiesLocal(frame)
            dth = controlModel.getInternalJointAngVelocitiesLocal()
            ddth_r = motion.getInternalJointAngAccelerationsLocal(frame)
            
            ddth_des[:] = getDesiredAngAccelerations(th_r, th, dth_r, dth, ddth_r, 1, 1)
            
            for i in range(stepsPerFrame):
                controlModel.setInternalJointAngAccelerationsLocal(ddth_des)
                controlModel.solveHybridDynamics()

                vpWorld.step()
                
            motionModel.update(motion[frame])
            p[:] = motion.getInternalJointPositionsGlobal(frame)
                
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
                
    pass
    test_getDesiredAngAccelerations()