# +-------------------------------------------------------------------------
# | main_PreprocessMcfg.py
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
    
    # torso : 10
    massMap['Hips'] += 2.
    massMap['Spine'] += 8.
    
    # head : 3
    massMap['Spine1'] += 3.
    
    # right upper arm : 2
    massMap['RightArm'] += 2.
    
    # left upper arm : 2
    massMap['LeftArm'] += 2.
    
    # right lower arm : 1
    massMap['RightForeArm'] = 1.
#    massMap['RightForeArm'] = 2.
    
    # left lower arm : 1
    massMap['LeftForeArm'] = 1.
#    massMap['LeftForeArm'] = 2.
    
    # right thigh : 7
    massMap['Hips'] += 2.
    massMap['RightUpLeg'] += 5.
    
    # left thigh : 7
    massMap['Hips'] += 2.
    massMap['LeftUpLeg'] += 5.
    
    # right shin : 5
    massMap['RightLeg'] += 5.
    
    # left shin : 5
    massMap['LeftLeg'] += 5.
    
    # right foot : 4
    massMap['RightFoot'] += 2.
    
    # left foot : 4
    massMap['LeftFoot'] += 2.
    
    return massMap
massMap = buildMassMap()

if __name__=='__main__':
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .9

    totalMass = 0.
    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]
        totalMass += node.mass
    print totalMass
        
    node = mcfg.getNode('Hips')
    node.length = .2
    node.width = .25
    
    node = mcfg.getNode('Spine1')
    node.length = .2
    node.offset = (0,0,0.1)
    
    node = mcfg.getNode('Spine')
    node.width = .22
    
    node = mcfg.getNode('RightFoot')
    node.length = .25
#    node.length = .27
#    node.offset = (0,0,0.01)
    node.width = .1
    node.geom = 'MyFoot1'
    
    node = mcfg.getNode('LeftFoot')
    node.length = .25
#    node.length = .27
#    node.offset = (0,0,0.01)
    node.width = .1
    node.geom = 'MyFoot1'

#    dir = './ppmotion/'
    dir = './icmotion_last/'
    outputName = 'mcfg'
    outputFile = open(dir+outputName, 'w')
    cPickle.dump(mcfg, outputFile)
    outputFile.close()
     
    print dir + outputName, 'done'
    print 'FINISHED'
