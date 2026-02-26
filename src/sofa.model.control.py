import os, math, numpy as np
import Sofa.Core, SofaRuntime

from scipy import signal
from controller import SoftBodyController

###

root = Sofa.Core.Node()
path = os.path.dirname(os.path.abspath(__file__))+'/plot/'

###

class FingerController(Sofa.Core.Controller):

	def __init__(self, *args, **kwargs):
		Sofa.Core.Controller.__init__(self,*args, **kwargs)

		self.node = kwargs['node']
		self.effector = kwargs['effector']
		# self.target = np.array([-1.0, -265.0, 0.0]) + np.array([16.0, 0.0, -13.0])    # Q1  (+x +z)
		# self.target = np.array([-1.0, -265.0, 0.0]) + np.array([-14.0, 0.0, -13.0])   # Q2   (-x +z)
		# self.target = np.array([-1.0, -265.0, 0.0]) + np.array([16.0, 0.0, -33.0])    # Q3   (+x -z)
		self.target = np.array([-1.0, -265.0, 0.0]) + np.array([-14.0, 0.0, -33.0])   # Q4   (-x -z)

		self.time = 0.0
		self.dt = self.node.dt.value

		self.controller = SoftBodyController(kp=2e-6, ki=2e-8, kd=0.0)


	def onAnimateBeginEvent(self,event):
		self.time = self.node.time.value

		print(f'Position Target:\t[{self.target[0]:.5f}, {self.target[1]:.5f}, {self.target[2]:.5f}]')
		print(f'Position Fingertip:\t[{self.effector[0][0]:.5f}, {self.effector[0][1]:.5f}, {self.effector[0][2]:.5f}]')
		p1, p2, p3, p4 = self.controller.calculate_pressure(np.array(self.effector[0]), self.target, self.dt)
		print(f"Pressure values:\t[{p1:.5f}, {p2:.5f}, {p3:.5f}, {p4:.5f}]\n")

		self.node.finger1.cavity1.SurfaceForceField.value[0] = p4  	# Q1 -> Q4 upper left   (+x +z)
		self.node.finger1.cavity2.SurfaceForceField.value[0] = p3  	# Q2 -> Q3 upper right  (-x +z)
		self.node.finger1.cavity3.SurfaceForceField.value[0] = p2  	# Q3 -> Q2 lower left   (+x -z)
		self.node.finger1.cavity4.SurfaceForceField.value[0] = p1   # Q4 -> Q1 lower right  (-x -z)


def createScene(rootNode):
	rootNode.gravity=[0, 0,-9810]
	rootNode.dt = 0.01  # Viscolasity should have a lower dt to show effect
	rootNode.name = 'rootNode'

	# Required plugins
	rootNode.addObject('RequiredPlugin', name='SofaPython3')
	rootNode.addObject('RequiredPlugin', name="Sofa.Component.Engine.Select")
	rootNode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.LinearSolver.Iterative")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Mapping.Linear")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Mass")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.ODESolver.Forward")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Setting")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.SolidMechanics.FEM.HyperElastic")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.SolidMechanics.FEM.Elastic")	
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.SolidMechanics.Spring")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.StateContainer")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Topology.Container.Dynamic")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Visual")
	rootNode.addObject("RequiredPlugin", name="Sofa.GL.Component.Rendering3D")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.AnimationLoop")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Constraint.Lagrangian.Solver")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.MechanicalLoad")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.LinearSolver.Direct")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Constraint.Lagrangian.Correction")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.Constraint.Projective")
	rootNode.addObject("RequiredPlugin", name="Sofa.Component.ODESolver.Backward")
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Model') # Needed to use components [BilateralLagrangianConstraint]  
	rootNode.addObject('RequiredPlugin', name='SoftRobots') # Needed to use components [SurfacePressureConstraint]  
	rootNode.addObject('RequiredPlugin', name='MultiThreading')

	# Visual
	rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
	rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')

	# Solvers and Loop
	rootNode.addObject('FreeMotionAnimationLoop', parallelCollisionDetectionAndFreeMotion="0", parallelODESolving="0", computeBoundingBox="0")
	rootNode.addObject('NNCGConstraintSolver', tolerance=1e-24, maxIterations=1000)

	###

	# ************************************************************************************************************************************************
	## Finger
	finger = rootNode.addChild('finger1')
	finger.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness="0.0", rayleighMass="0.0")
	finger.addObject('SparseLDLSolver', name="LinearSolver", template="CompressedRowSparseMatrixMat3x3d")

	finger.addObject('MeshVTKLoader', name='loader', filename="mesh/SPA.vtu", translation = [0, -115, 0], rotation=[90, 0, 90])
	finger.addObject('MechanicalObject', name='dofs', template='Vec3d', src = '@loader')
	finger.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	finger.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	finger.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	finger.addObject('UniformMass', totalMass="115e-6", src = '@topo')
	#finger.addObject('TetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 0.6)
	mu1 = 0.24203
	lamb = 0
	finger.addObject('TetrahedronHyperelasticityFEMForceField', template='Vec3d', name='FEM', src ='@topo', ParameterSet=str(mu1)+' '+str(lamb),materialName="StableNeoHookean")
	finger.addObject('LinearSolverConstraintCorrection')

	visual = finger.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [0, -115, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,0.25]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	# ************************************************************************************************************************************************
	## Cavity
	cavity1 = finger.addChild('cavity1')
	cavity1.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_upper_left.obj', translation = [0, 0, 0], rotation=[90, 0, 90])
	cavity1.addObject('MeshTopology', src='@loader', name='topo')
	cavity1.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity1.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity1.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	cavity2 = finger.addChild('cavity2')
	cavity2.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_upper_right.obj', translation = [0, 0, 0], rotation=[90, 0, 90])
	cavity2.addObject('MeshTopology', src='@loader', name='topo')
	cavity2.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity2.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity2.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	cavity3 = finger.addChild('cavity3')
	cavity3.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_lower_left.obj', translation = [0, 0, 0], rotation=[90, 0, 90])
	cavity3.addObject('MeshTopology', src='@loader', name='topo')
	cavity3.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity3.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity3.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	cavity4 = finger.addChild('cavity4')
	cavity4.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_lower_right.obj', translation = [0, 0, 0], rotation=[90, 0, 90])
	cavity4.addObject('MeshTopology', src='@loader', name='topo')
	cavity4.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity4.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity4.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	visual = cavity1.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_upper_left.obj', translation = [0, 0, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	visual = cavity2.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_upper_right.obj', translation = [0, 0, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	visual = cavity3.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_lower_left.obj', translation = [0, 0, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	visual = cavity4.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_lower_right.obj', translation = [0, 0, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	# ************************************************************************************************************************************************
	## Spine
	spine = rootNode.addChild('spine1')
	spine.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness=0.0, rayleighMass=0.0)
	spine.addObject('SparseLDLSolver', name="LinearSolver", template="CompressedRowSparseMatrixMat3x3d")

	spine.addObject('MeshVTKLoader', name='loader', filename="mesh/arm_support.vtu", translation = [0, -115, 0], rotation=[90, 0, 90])
	spine.addObject('MechanicalObject', name='dofs', src = '@loader', template='Vec3d')
	spine.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	spine.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	spine.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	spine.addObject('UniformMass', src = '@topo', totalMass="15e-6")
	spine.addObject('ParallelTetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 75.5)
	# spine.addObject('BoxROI', name='boxROI',box="1 20 -22 30 30 22", drawBoxes = True)
	spine.addObject('BoxROI', name='boxROI',box="-15 -95 -22 15 -85 22", drawBoxes = True)
	spine.addObject('FixedConstraint', indices = '@boxROI.indices')
	spine.addObject('LinearSolverConstraintCorrection')

	visual = spine.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [0, -115, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0.5,0,0,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	spine.addObject('BoxROI', name='thrustROI',box="-15 -240 11 15 -265 22", drawBoxes = True)
	# [Throttle (%), Thrust (N)]
	# (10, 0.142); (20, 0.568); (30, 1.278); (40, 2.272)
	spine.addObject('ConstantForceField', name = "CFF", listening = True, totalForce =[0,0,0], template="Vec3d", src= "@topo", indices = spine.thrustROI.indices.linkpath)

	constraint_finger = finger.addChild('constraint_finger')
	constraint_finger.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'		+'\n'	+ '0 0 -5'		+'\n'	+'0 5 0'	+'\n'	+'0 -5 0'	+ '\n'+ 
		'-30 0 5'	+'\n'	+ '-30 0 -5'	+'\n'	+'-30 5 0'	+'\n'	+'-30 -5 0'	+ '\n'+
		'-60 0 5'	+'\n'	+ '-60 0 -5'	+'\n'	+'-60 5 0'	+'\n'	+'-60 -5 0'	+ '\n'+
		'-90 0 5'	+'\n'	+'-90 0 -5'		+'\n'	+'-90 5 0'	+'\n'	+'-90 -5 0'	+ '\n'+
		'-120 0 5'	+'\n'	+'-120 0 -5'	+'\n'	+'-120 5 0'	+'\n'	+'-120 -5 0', translation = [0, -115, 0], rotation=[90, 0, 90])
	constraint_finger.addObject('BarycentricMapping')

	constraint_spine = spine.addChild('constraint_spine')
	constraint_spine.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'		+'\n'	+ '0 0 -5'		+'\n'	+'0 5 0'	+'\n'	+'0 -5 0'	+ '\n'+ 
		'-30 0 5'	+'\n'	+ '-30 0 -5'	+'\n'	+ '-30 5 0'	+'\n'	+ '-30 -5 0'+ '\n'+
		'-60 0 5'	+'\n'	+ '-60 0 -5'	+'\n'	+ '-60 5 0'	+'\n'	+ '-60 -5 0'+ '\n'+
		'-90 0 5'	+'\n'	+ '-90 0 -5'	+'\n'	+ '-90 5 0'	+'\n'	+ '-90 -5 0'+ '\n'+
		'-120 0 5'	+'\n'	+'-120 0 -5'	+'\n'	+'-120 5 0'	+'\n'	+'-120 -5 0', translation = [0, -115, 0], rotation=[90, 0, 90])
	constraint_spine.addObject('BarycentricMapping')

	rootNode.addObject('BilateralLagrangianConstraint', name = 'vincolo', template="Vec3d", object1 = "@finger1/constraint_finger/position", object2 = "@spine1/constraint_spine/position", 
		first_point ="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19", 
		second_point="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19")

	effector = spine.addChild('Effector')
	effectorMO = effector.addObject('MechanicalObject', name='dofs', template='Vec3',position=[-1.0,-150.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[0, -115, 0], rotation = [0, 0, 0])
	effector.addObject('BarycentricMapping')

	spine.addObject(FingerController(node=rootNode, effector = rootNode.getChild('spine1').Effector.dofs.position.value))

	return rootNode
