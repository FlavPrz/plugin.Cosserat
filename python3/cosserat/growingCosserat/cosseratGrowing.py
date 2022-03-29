# -*- coding: utf-8 -*-
"""
    Cosserat class in SofaPython3.
"""

#__authors__ = "flavie"
#__contact__ = ""
#__version__ = "1.0.0"
#__copyright__ = ""
#__date__ = ""

from dataclasses import dataclass
import math
from math import cos, sin
import Sofa
from usefulFunctions import buildEdges, pluginList, BuildCosseratGeometry

cosserat_config = {'init_pos': [0., 0., 0.], 'tot_length': 6, 'nbSectionS': 7,
                   'nbFramesF': 24, 'buildCollisionModel': 1, 'beamMass': 0.22}


class Cosserat(Sofa.Prefab):
    """ActuatedArm is a reusable sofa model of a S90 servo motor and the tripod actuation arm.
           Parameters:
               -parent:        node where the ServoArm will be attached
                - translation the position in space of the structure
                - eulerRotation the orientation of the structure
                - attachingTo (MechanicalObject)    a rest shape force field will constraint the object
                                                 to follow arm position
           Structure:
           Node : {
                name : 'Cosserat'
                Node0 MechanicalObject :     // Rigid position of the base of the beam
                Node1 MechanicalObject :    // Vec3d, The rate angular composed of the twist and the bending along y and z
                Node1 ForceField          //
                    MechanicalObject     //  The child of the two precedent nodes, Rigid positions
                    Cosserat Mapping  //  it allow the transfer from the local to the global frame
            }
    """
    properties = [
        {'name': 'name', 'type': 'string', 'help': 'Node name', 'default': 'Cosserat'},
        {'name': 'position', 'type': 'Rigid3d::VecCoord', 'help': 'Cosserat base position',
         'default': [[0., 0., 0., 0, 0, 0, 1.]]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Cosserat base Rotation', 'default': [0., 0., 0.]},
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Cosserat base Rotation', 'default': [0., 0., 0.]},
        {'name': 'youngModulus', 'type': 'double', 'help': 'Beam Young modulus', 'default': 1.e6},
        {'name': 'poissonRatio', 'type': 'double', 'help': 'Beam poisson ratio', 'default': 0.4},
        {'name': 'shape', 'type': 'string', 'help': 'beam section', 'default': "circular"},
        {'name': 'radius', 'type': 'double', 'help': 'the radius in case of circular section', 'default': 1.0},
        {'name': 'length_Y', 'type': 'double', 'help': 'the radius in case of circular section', 'default': 1.0},
        {'name': 'length_Z', 'type': 'double', 'help': 'the radius in case of circular section', 'default': 1.0},
        {'name': 'attachingToLink', 'type': 'string', 'help': 'a rest shape force field will constraint the object '
                                                              'to follow arm position', 'default': '1'}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)
        self.cosseratGeometry = kwargs['cosseratGeometry']
        self.beamMass = self.cosseratGeometry['beamMass']
        self.parent = kwargs.get('parent', None)

        if self.parent.hasObject("EulerImplicitSolver") is False:
            self.solverNode = self.addSolverNode()
        else:
            self.solverNode = self.parent

        self.rigidBaseNode = self.addRigidBaseNode()
        [positionS, curv_abs_inputS, longeurS, framesF, curv_abs_outputF, self.frames3D] = \
            BuildCosseratGeometry(self.cosseratGeometry)

        # \ -> ligne a droite

        positionSM = []

        for i, pos in enumerate(positionS):
            if i==2:
                pos[0] = 0
                pos[1] = 0
                pos[2] = -7.0/6.0*3.14
            positionSM.append(pos)

        self.cosseratCoordinateNode = self.addCosseratCoordinate(positionSM, longeurS) #changement positionS par positionSM
        self.cosseratFrame = self.addCosseratFrame(framesF, curv_abs_inputS, curv_abs_outputF)
        print(f'=== > {curv_abs_inputS}')

    def init(self):
        pass

    ################################### Structure ###################################
    ##   ° SolverNode
    #        -EulerImplicit
    #        -Solver
    #        -Generic Constrain Correction

    #            °Rigid base
    #                -Rigid MechanicalObject
    #                -Spring
    #            °CosseratCoordinate
    #                -Cosserat Coordinate Mechanical object Vec3
    #                -Beam Hook Law
    #
    #                °Cosserat sofa frame
    #                    - Frames Mo
    #                    -Cosserat Mapping Base Index
    ##
    ################################################################################



    def addSolverNode(self):

        #solverNode_i = self.addChild('solver'+str(i)+'Node')
        solverNode = self.addChild('solverNode')
        solverNode.addObject('EulerImplicitSolver', rayleighStiffness="0.2", rayleighMass='0.1')
        solverNode.addObject('SparseLDLSolver', name='solver', template="CompressedRowSparseMatrixd")
        solverNode.addObject('GenericConstraintCorrection')


        return solverNode

    def addRigidBaseNode(self):

        for i in range(10):

            rigidBaseNode_i = self.solverNode.addChild('rigidBase'+str(i)+'Node')

            angle = 2 * i * math.pi / 10
            trans=[0, math.cos(angle), math.sin(angle)]
            rot=[angle * 180 / math.pi, 0, 0]
            positions = []
            for pos in self.position.value:
                _pos = [p for p in pos]
                positions.append(_pos)

            rigidBaseNode_i.addObject('MechanicalObject', template='Rigid3d', name="RigidBaseMO",
                                    showObjectScale=0.2, translation=trans,
                                    position=positions, rotation=rot, showObject=1)


            if int(self.attachingToLink.value):
                rigidBaseNode_i.addObject('RestShapeSpringsForceField', name='spring',
                                        stiffness=1e8, angularStiffness=1.e8, external_points=0,
                                        mstate="@RigidBaseMO", points=0, template="Rigid3d")

        return rigidBaseNode_i

    def addCosseratCoordinate(self, positionS, longeurS):

        for i in range (10):
            cosseratCoordinateNode_i = self.solverNode.addChild('cosseratCoordinate'+str(i)+'Node')
            cosseratCoordinateNode_i.addObject('MechanicalObject', template='Vec3d', name='cosseratCoordinateMO',position=positionS,showIndices=0)
            print('+++++ for pelo position S numero'+ str(i))
            print(positionS)


        return cosseratCoordinateNode_i

    def addCosseratFrame(self, framesF, curv_abs_inputS, curv_abs_outputF):
        for i in range(10):

            cosseratInSofaFrameNode_i = getattr(self.solverNode,'rigidBase'+str(i)+'Node').addChild('cosseratInSofaFrame'+str(i)+'Node')
            getattr(self.solverNode, 'cosseratCoordinate'+str(i)+'Node').addChild(cosseratInSofaFrameNode_i)
            framesMO = cosseratInSofaFrameNode_i.addObject('MechanicalObject', template='Rigid3d',
                                                         name="FramesMO", position=framesF,
                                                         showObject=1, showObjectScale=0.1)

            cosseratInSofaFrameNode_i.addObject('DiscreteCosseratMapping', curv_abs_input=curv_abs_inputS,
                                              curv_abs_output=curv_abs_outputF, name='cosseratMapping',
                                              input1=getattr(self.solverNode, 'cosseratCoordinate'+str(i)+'Node').cosseratCoordinateMO.getLinkPath(),
                                              input2=getattr(self.solverNode,'rigidBase'+str(i)+'Node').RigidBaseMO.getLinkPath(),
                                              output=framesMO.getLinkPath(), debug=0, radius=0)

        return cosseratInSofaFrameNode_i


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='plugins', pluginName=[pluginList,
                                                                     ['SofaEngine', 'SofaLoader', 'SofaSimpleFem',
                                                                      'SofaExporter']])
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels hideCollisionModels '
                                                   'hideBoundingCollisionModels hireForceFields '
                                                   'hideInteractionForceFields hideWireframe')
    rootNode.findData('dt').value = 0.01
    rootNode.findData('gravity').value = [0., -9.81, 0.]
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=5e2)
    rootNode.addObject('EulerImplicitSolver', rayleighStiffness="0.2", rayleighMass='0.1')
    rootNode.addObject('SparseLDLSolver', name='solver', template="CompressedRowSparseMatrixd")
    rootNode.addObject('GenericConstraintCorrection')

    cosserat = rootNode.addChild(
        Cosserat(parent=rootNode, cosseratGeometry=cosserat_config, name="cosserat", radius = 1))

    return rootNode