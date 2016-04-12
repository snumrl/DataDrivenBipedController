# +-------------------------------------------------------------------------
# | main_PreprocessMcfg_simbicon.py
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

import psyco; psyco.full()
import cPickle

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')
import Simulator.ysPhysConfig as ypc

def buildMassMap():
    massMap = {}
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
    
    # Trunk   29.27
    massMap['Spine'] += 29.27

    # Pelvis  16.61
    massMap['Hips'] += 16.61
    
    # Head    5.89
    massMap['Spine1'] += 5.89
    
    # Thigh   8.35
    massMap['RightUpLeg'] += 8.35
    massMap['LeftUpLeg'] += 8.35
    
    # Shank   4.16
    massMap['RightLeg'] += 4.16
    massMap['LeftLeg'] += 4.16
    
    # Foot    1.34
    massMap['RightFoot'] += 1.34
    massMap['LeftFoot'] += 1.34
    
    # Arm     2.79
    massMap['RightArm'] += 2.79
    massMap['LeftArm'] += 2.79
    
    # Forearm 1.21
    # Hand    0.55
    massMap['RightForeArm'] = 1.21 + 0.55
    massMap['LeftForeArm'] = 1.21 + 0.55
    return massMap
massMap = buildMassMap()

if __name__=='__main__':
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = 1.

    totalMass = 0.
    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]
        totalMass += node.mass
    print totalMass
        
    node = mcfg.getNode('Hips')
    node.length = .2
    node.width = .25
    node.density = 1500
    
    node = mcfg.getNode('Spine1')
    node.length = .2
    node.offset = (0,0,0.1)
    
    node = mcfg.getNode('Spine')
    node.width = .22
    node.density = 2000
    
    node = mcfg.getNode('RightFoot')
    node.length = .25
#    node.length = .25*.9
    node.width = .1
    node.geom = 'MyFoot1'
    
    node = mcfg.getNode('LeftFoot')
    node.length = .25
#    node.length = .25*.9
    node.width = .1
    node.geom = 'MyFoot1'
    
    # for fixed_foot
    node = mcfg.getNode('LeftLeg')
    node.boneRatio = .9
    node = mcfg.getNode('RightLeg')
    node.boneRatio = .9

    dir = './'
#    outputName = 'mcfg_simbicon'
    outputName = 'mcfg_simbicon_fixed_foot'
    outputFile = open(dir+outputName, 'w')
    cPickle.dump(mcfg, outputFile)
    outputFile.close()
     
    print dir + outputName, 'done'
    print 'FINISHED'
