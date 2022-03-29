# -*- coding: utf-8 -*-
"""
    Cosserat class in SofaPython3.
"""

__authors__ = "younesssss"
__contact__ = "adagolodjo@protonmail.com"
__version__ = "1.0.0"
__copyright__ = "(c) 2021,Inria"
__date__ = "October, 26 2021"

from dataclasses import dataclass
import math
from math import cos, sin
import Sofa
from usefulFunctions import buildEdges, pluginList, BuildCosseratGeometry

cosserat_config = {'init_pos': [0., 0., 0.], 'tot_length': 6, 'nbSectionS': 7,
                   'nbFramesF': 24, 'buildCollisionModel': 1, 'beamMass': 0.22}

cosserat_config2 = {'init_pos': [0., 0., 0.], 'tot_length': 6, 'nbSectionS': 7,
                   'nbFramesF': 24, 'buildCollisionModel': 1, 'beamMass': 0.22}

cosserat_config3 = {'init_pos': [6., 0., 0.], 'tot_length': 1, 'nbSectionS': 3,
                   'nbFramesF': 3, 'buildCollisionModel': 1, 'beamMass': 0.22}


# @dataclass
def addEdgeCollision(parentNode, position3D, edges):
    collisInstrumentCombined = parentNode.addChild('collisInstrumentCombined')
    collisInstrumentCombined.addObject('EdgeSetTopologyContainer', name="collisEdgeSet", position=position3D,
                                       edges=edges)
    collisInstrumentCombined.addObject('EdgeSetTopologyModifier', name="collisEdgeModifier")
    collisInstrumentCombined.addObject('MechanicalObject', name="CollisionDOFs")
    collisInstrumentCombined.addObject('LineCollisionModel', bothSide="1", group='2')
    collisInstrumentCombined.addObject('PointCollisionModel', bothSide="1", group='2')
    collisInstrumentCombined.addObject('IdentityMapping', name="mapping")
    return collisInstrumentCombined


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

        positionSM = []

        for i, pos in enumerate(positionS):
            if i==4:
                pos[0] = 0
                pos[1] = 0
                pos[2] = -7.0/6.0*3.14
            positionSM.append(pos)

        self.cosseratCoordinateNode = self.addCosseratCoordinate(positionSM, longeurS) #changement positionS par positionSM
        self.cosseratFrame = self.addCosseratFrame(framesF, curv_abs_inputS, curv_abs_outputF)
        print(f'=== > {curv_abs_inputS}')

    def init(self):
        pass

    def addCollisionModel(self):
        tab_edges = buildEdges(self.frames3D)
        return addEdgeCollision(self.cosseratFrame, self.frames3D, tab_edges)

    def addSolverNode(self):
        solverNode = self.addChild('solverNode')
        solverNode.addObject('EulerImplicitSolver', rayleighStiffness="0.2", rayleighMass='0.1')
        solverNode.addObject('SparseLDLSolver', name='solver', template="CompressedRowSparseMatrixd")
        solverNode.addObject('GenericConstraintCorrection')
        return solverNode

    def addRigidBaseNode(self):
        rigidBaseNode = self.solverNode.addChild('rigidBase')

        trans = [t for t in self.translation.value]
        rot = [r for r in self.rotation.value]
        positions = []
        for pos in self.position.value:
            _pos = [p for p in pos]
            positions.append(_pos)
        print("-----------------------------")
        print(rot)
        rigidBaseNode.addObject('MechanicalObject', template='Rigid3d', name="RigidBaseMO",
                                showObjectScale=0.2, translation=trans,
                                position=positions, rotation=rot, showObject=1)
        # one can choose to set this to false and directly attach the beam base
        # to a control object in order to be able to drive it.
        if int(self.attachingToLink.value):
            rigidBaseNode.addObject('RestShapeSpringsForceField', name='spring',
                                    stiffness=1e8, angularStiffness=1.e8, external_points=0,
                                    mstate="@RigidBaseMO", points=0, template="Rigid3d")
        return rigidBaseNode

    def addCosseratCoordinate(self, positionS, longeurS):
        cosseratCoordinateNode = self.solverNode.addChild('cosseratCoordinate')
        cosseratCoordinateNode.addObject('MechanicalObject',
                                         template='Vec3d', name='cosseratCoordinateMO',
                                         position=positionS,
                                         showIndices=0)
        cosseratCoordinateNode.addObject('BeamHookeLawForceField', crossSectionShape=self.shape.value,
                                         length=longeurS, youngModulus=self.youngModulus.value,
                                         poissonRatio=self.poissonRatio.value,
                                         radius=self.radius.value,
                                         lengthY=self.length_Y.value, lengthZ=self.length_Z.value)
        return cosseratCoordinateNode

    def addCosseratFrame(self, framesF, curv_abs_inputS, curv_abs_outputF):

        cosseratInSofaFrameNode = self.rigidBaseNode.addChild('cosseratInSofaFrameNode')
        self.cosseratCoordinateNode.addChild(cosseratInSofaFrameNode)


        framesMO = cosseratInSofaFrameNode.addObject('MechanicalObject', template='Rigid3d',
                                                     name="FramesMO", position=framesF,
                                                     showObject=1, showObjectScale=0.1)



        cosseratInSofaFrameNode.addObject('DiscreteCosseratMapping', curv_abs_input=curv_abs_inputS,
                                          curv_abs_output=curv_abs_outputF, name='cosseratMapping',
                                          input1=self.cosseratCoordinateNode.cosseratCoordinateMO.getLinkPath(),
                                          input2=self.rigidBaseNode.RigidBaseMO.getLinkPath(),
                                          output=framesMO.getLinkPath(), debug=0, radius=0)

        return cosseratInSofaFrameNode


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
    rootNode.addObject('Camera', position="-35 0 280", lookAt="0 0 0")

    solverNode = rootNode.addChild('solverNode')
    solverNode.addObject('EulerImplicitSolver', rayleighStiffness="0.2", rayleighMass='0.1')
    solverNode.addObject('SparseLDLSolver', name='solver', template="CompressedRowSparseMatrixd")
    solverNode.addObject('GenericConstraintCorrection')

    beamNodeList=[]
    for i in range(10):
        angle = 2*i*math.pi/10
        beamNode_i = solverNode.addChild('beam'+str(i)+'Node')
        beamNodeList.append(beamNode_i)
        cosserat = solverNode.addChild(
            Cosserat(parent=beamNode_i, cosseratGeometry=cosserat_config, name="cosserat", radius=1, translation=[0, cos(angle), sin(angle)], rotation=[angle*180/math.pi, 0, 0]))

    return rootNode
