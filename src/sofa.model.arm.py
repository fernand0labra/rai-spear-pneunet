import os, math, numpy as np
import Sofa.Core, SofaRuntime

from scipy import signal
import math 
import numpy as np
###

root = Sofa.Core.Node()
path = os.path.dirname(os.path.abspath(__file__))+'/plot/'

###

class FingerController(Sofa.Core.Controller):

	def __init__(self, *args, **kwargs):
		Sofa.Core.Controller.__init__(self,*args, **kwargs)

		self.time = 0.0

		self.idx = kwargs['idx']  # Index of the finger

		self.pos1 = kwargs['pos1']
		self.pos2 = kwargs['pos2']

		self.node = kwargs['node']

		self.X = [];  self.Y = [];  self.Z = []

		dt = self.node.dt.value

		self.pressure1 = 0.00065
		self.pressure2 = 0.00065
		self.pressure_max = 0.0013  # Maximum pressure value

		self.coeffin = (self.pos1[0][1]-self.pos2[0][1])/(self.pos1[0][0]-self.pos2[0][0])
		self.anglein = (math.atan(self.coeffin))*180/math.pi 

		self.coeffin1 = (self.pos1[0][2]-self.pos2[0][2])/(self.pos1[0][0]-self.pos2[0][0])
		self.anglein1 = (math.atan(self.coeffin))*180/math.pi 


	def onAnimateBeginEvent(self,event):
		self.time = self.node.time.value

		## ANGLE X-Y
		self.coeff = (self.pos1[0][1]-self.pos2[0][1])/(self.pos1[0][0]-self.pos2[0][0])
		
		if self.coeff < 0:
			self.angle = (math.atan2((self.pos1[0][1]-self.pos2[0][1]), (self.pos1[0][0]-self.pos2[0][0]) ))*180/math.pi - 45

		if self.coeff > 0:
			self.angle = (math.atan(self.coeff))*180/math.pi - 45

		## ANGLE X-Z	

		self.coeff1 = (self.pos1[0][2]-self.pos2[0][2])/(self.pos1[0][0]-self.pos2[0][0]) 
		
		if self.coeff1 < 0:
			self.angle1 = (math.atan2((self.pos1[0][2]-self.pos2[0][2]), (self.pos1[0][0]-self.pos2[0][0]) ))*180/math.pi - self.anglein1

		if self.coeff1 > 0:
			self.angle1 = (math.atan(self.coeff1))*180/math.pi - self.anglein1

		# if not ((self.pressure1 == self.pressure_max and self.pressure2 == 0.0) or (self.pressure1 == 0.0 and self.pressure2 == self.pressure_max)):

		self.pressure1 = 0.00065 + 0.00065*np.sin(2*math.pi*0.05*self.time)
		self.pressure2 = 0.00065 - 0.00065*np.sin(2*math.pi*0.05*self.time)

		self.node.getChild(f'finger{self.idx}').cavity1.SurfaceForceField.value[0] = self.pressure1 if self.idx % 2 == 0 else self.pressure2
		self.node.getChild(f'finger{self.idx}').cavity2.SurfaceForceField.value[0] = self.pressure2 if self.idx % 2 == 0 else self.pressure1

		print('Pressure Cavity 1: ' + str(self.pressure1))
		print('Pressure Cavity 2: ' + str(self.pressure2))
		print('Angle X-Y: '+ str(self.angle))
		print('Angle X-Z: '+ str(self.angle1))
		print('')

		self.X.append(self.pos2[0][1]);  self.Y.append(self.pos2[0][0]);  self.Z.append(self.pos2[0][2])

		if len(self.X) == 2000:
			fileX = open('./x.txt', 'w')
			for x in self.X:
				fileX.write(f'{x:.4f}\n')
			fileX.flush();  fileX.close()

			fileY = open('./y.txt', 'w')
			for y in self.Y:
				fileY.write(f'{y:.4f}\n')
			fileY.flush();  fileY.close()

			fileZ = open('./z.txt', 'w')
			for z in self.Z:
				fileZ.write(f'{z:.4f}\n')
			fileZ.flush();  fileZ.close()


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
	rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields hideInteractionForceFields hideWireframe')
	rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')
	# rootNode.addObject('BackgroundSetting', color='1 1 1')

	# # Collision
	# rootNode.addObject('CollisionPipeline')
	# rootNode.addObject('ParallelBVHNarrowPhase')
	# rootNode.addObject('ParallelBruteForceBroadPhase')
	# rootNode.addObject('CollisionResponse', response='FrictionContactConstraint')
	# rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5)

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

	# collision = finger.addChild('collision')
	# collision.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [0, -115, 0], rotation=[90, 0, 90], triangulate=True)
	# collision.addObject('MeshTopology', src='@loader')
	# collision.addObject('MechanicalObject', template='Vec3d')
	# collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	# collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	# collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	# collision.addObject('BarycentricMapping')

	visual = finger.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [0, -115, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,0.25]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	# ************************************************************************************************************************************************
	## Cavity
	cavity1 = finger.addChild('cavity1')
	cavity1.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_left.obj', translation = [0, -115, 0], rotation=[90, 0, 90])
	cavity1.addObject('MeshTopology', src='@loader', name='topo')
	cavity1.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity1.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity1.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	cavity2 = finger.addChild('cavity2')
	cavity2.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_right.obj', translation = [0, -115, 0], rotation=[90, 0, 90])
	cavity2.addObject('MeshTopology', src='@loader', name='topo')
	cavity2.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity2.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity2.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	visual = cavity1.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_left.obj', translation = [0, -115, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	visual = cavity2.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_right.obj', translation = [0, -115, 0], rotation=[90, 0, 90])
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

	# collision = spine.addChild('collision')
	# collision.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [0, -115, 0], rotation=[90, 0, 90], triangulate=True)
	# collision.addObject('MeshTopology', src='@loader')
	# collision.addObject('MechanicalObject', template='Vec3d')
	# collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	# collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	# collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	# collision.addObject('BarycentricMapping')

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

	# effector = spine.addChild('Effector')
	# effectorMO = effector.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0,-150.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[0, -115, 0], rotation = [0, 0, 0])
	# effector.addObject('BarycentricMapping')

	# origin = spine.addChild('Origin')
	# originMO = origin.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0, 30.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[0, -115, 0], rotation = [0, 0, 0])
	# origin.addObject('BarycentricMapping')	
	# spine.addObject(FingerController(node=rootNode, pos1= origin.dofs.position.value, pos2 = rootNode.getChild('spine1').Effector.dofs.position.value , idx=1))

	return rootNode
