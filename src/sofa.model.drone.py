import os, math, numpy as np
import Sofa.Core, Sofa.Gui, SofaRuntime

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

		dt = self.node.dt.value

		# Cavity 1 starts decreasing from max pressure
		# Cavity 2 starts increasing from min pressure
		self.pressure1 = 0.00065
		self.pressure2 = 0.00065

		self.pressure_inc = 0.00005 # Pressure increment per time step
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

		if not ((self.pressure1 == self.pressure_max and self.pressure2 == 0.0) or (self.pressure1 == 0.0 and self.pressure2 == self.pressure_max)):
			self.pressure1 = 0.00065 + 0.00065*np.sin(2*math.pi*0.05*self.time)
			self.pressure2 = 0.00065 - 0.00065*np.sin(2*math.pi*0.05*self.time)

			self.node.getChild(f'finger{self.idx}').cavity1.SurfaceForceField.value[0] = self.pressure1 if self.idx % 2 == 0 else self.pressure2
			self.node.getChild(f'finger{self.idx}').cavity2.SurfaceForceField.value[0] = self.pressure2 if self.idx % 2 == 0 else self.pressure1

		if f'finger{self.idx}' == 'finger1':
			print('Pressure Cavity 1: ' + str(self.pressure1))
			print('Pressure Cavity 2: ' + str(self.pressure2))
			print('Angle X-Y: '+ str(self.angle))
			print('Angle X-Z: '+ str(self.angle1))
			print('')


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
	rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Model') # Needed to use components [BilateralInteractionConstraint]
	rootNode.addObject('RequiredPlugin', name='MultiThreading') # Needed to use components [ParallelBVHNarrowPhase, ParallelBruteForceBroadPhase]
	rootNode.addObject('RequiredPlugin', name='SoftRobots') # Needed to use components [SurfacePressureConstraint]  

	# Visual
	rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields hideInteractionForceFields hideWireframe')
	rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')

	# Collision
	rootNode.addObject('CollisionPipeline')
	rootNode.addObject('ParallelBVHNarrowPhase')
	rootNode.addObject('ParallelBruteForceBroadPhase')
	rootNode.addObject('CollisionResponse', response='FrictionContactConstraint')
	rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5)

	# Solvers and Loop
	rootNode.addObject('FreeMotionAnimationLoop', computeBoundingBox="0")
	# v23.06: NNCGConstraintSolver (SoftRobots) does not exist yet; GenericConstraintSolver
	# is the SOFA-core equivalent and accepts the same tolerance/maxIterations parameters.
	rootNode.addObject('GenericConstraintSolver', tolerance=1e-24, maxIterations=1000)

	dronebox = rootNode.addChild('dronebox')
	dronebox.addObject('EulerImplicitSolver', name='odesolver')
	dronebox.addObject('SparseLDLSolver', name='linearSolver', template="CompressedRowSparseMatrixMat3x3d")
	dronebox.addObject('MechanicalObject', template='Rigid3d')
	dronebox.addObject('UniformMass', totalMass='1000e-6')
	dronebox.addObject('FixedConstraint', indices=[0])  # Anchor the rigid hub in place (matches the world-fixed arm bases)
	dronebox.addObject('LinearSolverConstraintCorrection')
	
	collision = dronebox.addChild('collision')
	collision.addObject('MeshOBJLoader', name='loader', filename='mesh/objects/drone_box.obj', rotation=[90, 0, 0], triangulate=True)
	collision.addObject('MeshTopology', src='@loader')
	collision.addObject('MechanicalObject', template='Vec3d')
	collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	collision.addObject('RigidMapping')

	visual = dronebox.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/objects/drone_box.obj', rotation=[90, 0, 0])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,1.0]) # Color model 0-1 (RGB alpha)
	visual.addObject('RigidMapping')

	###

	# ************************************************************************************************************************************************
	## Finger
	finger = rootNode.addChild('finger1')
	finger.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness="0.0", rayleighMass="0.0")
	finger.addObject('SparseLDLSolver', name="LinearSolver")

	finger.addObject('MeshVTKLoader', name='loader', filename="mesh/SPA.vtu", translation = [0, -115, 0], rotation=[90, 0, 90])
	finger.addObject('MechanicalObject', name='dofs', template='Vec3d', src = '@loader')
	finger.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	finger.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	finger.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	finger.addObject('UniformMass', totalMass="115e-6", src = '@topo')
	#finger.addObject('TetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 0.6)
	mu1 = 0.24203
	lamb = 0
	finger.addObject('TetrahedronHyperelasticityFEMForceField', template='Vec3d', name='FEM', src ='@topo', ParameterSet=str(mu1)+' '+str(lamb),materialName="NeoHookean")
	finger.addObject('LinearSolverConstraintCorrection')

	collision = finger.addChild('collision')
	collision.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [0, -115, 0], rotation=[90, 0, 90], triangulate=True)
	collision.addObject('MeshTopology', src='@loader')
	collision.addObject('MechanicalObject', template='Vec3d')
	collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	collision.addObject('BarycentricMapping')

	visual = finger.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [0, -115, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,0.25]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	# ************************************************************************************************************************************************
	## Rotor
	rotor = rootNode.addChild('rotor1')
	rotor.addObject('EulerImplicitSolver', name='odesolver')
	rotor.addObject('SparseLDLSolver', name='linearSolver', template="CompressedRowSparseMatrixMat3x3d")
	rotor.addObject('MechanicalObject', template='Rigid3d')
	rotor.addObject('UniformMass', totalMass='1e-10')
	rotor.addObject('LinearSolverConstraintCorrection')

	visual = rotor.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/rotor.obj', translation=[-1, -255, 5], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,1.0]) # Color model 0-1 (RGB alpha)
	visual.addObject('RigidMapping')

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
	spine.addObject('SparseLDLSolver', name="LinearSolver")

	spine.addObject('MeshVTKLoader', name='loader', filename="mesh/arm_support.vtu", translation = [0, -115, 0], rotation=[90, 0, 90])
	spine.addObject('MechanicalObject', name='dofs', src = '@loader', template='Vec3d')
	spine.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	spine.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	spine.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	spine.addObject('UniformMass', src = '@topo', totalMass="15e-6")
	spine.addObject('TetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 75.5)
	# spine.addObject('BoxROI', name='boxROI',box="1 20 -22 30 30 22", drawBoxes = True)
	spine.addObject('BoxROI', name='boxROI',box="-15 -95 -22 16 -85 22", drawBoxes = True)
	spine.addObject('FixedConstraint', indices = '@boxROI.indices')
	spine.addObject('LinearSolverConstraintCorrection')

	collision = spine.addChild('collision')
	collision.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [0, -115, 0], rotation=[90, 0, 90], triangulate=True)
	collision.addObject('MeshTopology', src='@loader')
	collision.addObject('MechanicalObject', template='Vec3d')
	collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	collision.addObject('BarycentricMapping')

	visual = spine.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [0, -115, 0], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0.5,0,0,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	constraint_finger = finger.addChild('constraint_finger')
	constraint_finger.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'+'\n'+ '0 0 -5'+'\n'+'0 5 0'+'\n'+'0 -5 0'+ '\n' + 
		'-30 0 5'+'\n'+ '-30 0 -5'+'\n'+'-30 5 0'+'\n'+'-30 -5 0'+ '\n'+
		'-60 0 5'+'\n'+ '-60 0 -5'+'\n'+'-60 5 0'+'\n'+'-60 -5 0'+ '\n'+
		'-90 0 5'+'\n'+'-90 0 -5'+'\n'+'-90 5 0'+'\n'+'-90 -5 0'+ '\n'+
		'-120 0 5'+'\n'+'-120 0 -5'+'\n'+'-120 5 0'+'\n'+'-120 -5 0', translation = [0, -115, 0], rotation=[90, 0, 90])
	constraint_finger.addObject('BarycentricMapping')

	constraint_spine = spine.addChild('constraint_spine1')
	constraint_spine.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'+'\n'+ '0 0 -5'+'\n'+'0 5 0'+'\n'+'0 -5 0'+ '\n' + 
		'-30 0 5'+'\n'+ '-30 0 -5'+'\n'+ '-30 5 0'+'\n'+ '-30 -5 0'+ '\n'+
		'-60 0 5'+'\n'+ '-60 0 -5'+'\n'+ '-60 5 0'+'\n'+ '-60 -5 0'+ '\n'+
		'-90 0 5'+'\n'+ '-90 0 -5'+'\n'+ '-90 5 0'+'\n'+ '-90 -5 0'+ '\n'+
		'-120 0 5'+'\n'+'-120 0 -5'+'\n'+'-120 5 0'+'\n'+'-120 -5 0', translation = [0, -115, 0], rotation=[90, 0, 90])
	constraint_spine.addObject('BarycentricMapping')

	rootNode.addObject('BilateralInteractionConstraint', name = 'blc_finger1_spine1', template="Vec3d", object1 = "@finger1/constraint_finger/position", object2 = "@spine1/constraint_spine1/position", 
		first_point="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19", 
		second_point="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19")

	constraint_spine = spine.addChild('constraint_spine2')
	constraint_spine.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'-8 10 8'+'\n'+ '8 10 8'+'\n'+'-8 10 -2'+'\n'+'8 10 -2'+ '\n' + 
		'-8 0 8'+'\n'+ '8 0 8'+'\n'+'-8 0 -2'+'\n'+'8 0 -2'+ '\n' +
		'-8 -10 8'+'\n'+ '8 -10 8'+'\n'+'-8 -10 -2'+'\n'+'8 -10 -2', translation = [0, -255, 0], rotation=[0, 0, 0])
	constraint_spine.addObject('BarycentricMapping')

	constraint_rotor = rotor.addChild('constraint_rotor')
	constraint_rotor.addObject('MechanicalObject', name='position', position =
		'-8 10 8'+'\n'+ '8 10 8'+'\n'+'-8 10 -2'+'\n'+'8 10 -2'+ '\n' + 
		'-8 0 8'+'\n'+ '8 0 8'+'\n'+'-8 0 -2'+'\n'+'8 0 -2'+ '\n' +
		'-8 -10 8'+'\n'+ '8 -10 8'+'\n'+'-8 -10 -2'+'\n'+'8 -10 -2', translation = [0, -255, 0], rotation=[0, 0, 0])
	constraint_rotor.addObject('RigidMapping')

	rootNode.addObject('BilateralInteractionConstraint', name = 'blc_spine1_rotor1', template="Vec3d", object1 = "@spine1/constraint_spine2/position", object2 = "@rotor1/constraint_rotor/position", 
		first_point="0 1 2 3 4 5 6 7 8 9 10 11", 
		second_point="0 1 2 3 4 5 6 7 8 9 10 11")

	effector = spine.addChild('Effector')
	effectorMO = effector.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0,-150.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[0, -115, 0], rotation = [0, 0, 0])
	effector.addObject('BarycentricMapping')

	origin = spine.addChild('Origin')
	originMO = origin.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0, 30.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[0, -115, 0], rotation = [0, 0, 0])
	origin.addObject('BarycentricMapping')	
	spine.addObject(FingerController(node=rootNode, pos1= origin.dofs.position.value, pos2 = rootNode.getChild('spine1').Effector.dofs.position.value , idx=1))


	###


	# ************************************************************************************************************************************************
	## Finger
	finger = rootNode.addChild('finger2')
	finger.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness="0.0", rayleighMass="0.0")
	finger.addObject('SparseLDLSolver', name="LinearSolver")

	finger.addObject('MeshVTKLoader', name='loader', filename="mesh/SPA.vtu", translation = [115, 0, 0], rotation=[90, 0, 180])
	finger.addObject('MechanicalObject', name='dofs', template='Vec3d', src = '@loader')
	finger.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	finger.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	finger.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	finger.addObject('UniformMass', totalMass="115e-6", src = '@topo')
	#finger.addObject('TetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 0.6)
	mu1 = 0.24203
	lamb = 0
	finger.addObject('TetrahedronHyperelasticityFEMForceField', template='Vec3d', name='FEM', src ='@topo', ParameterSet=str(mu1)+' '+str(lamb),materialName="NeoHookean")
	finger.addObject('LinearSolverConstraintCorrection')

	collision = finger.addChild('collision')
	collision.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [115, 0, 0], rotation=[90, 0, 180], triangulate=True)
	collision.addObject('MeshTopology', src='@loader')
	collision.addObject('MechanicalObject', template='Vec3d')
	collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	collision.addObject('BarycentricMapping')

	visual = finger.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [115, 0, 0], rotation=[90, 0, 180])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,0.25]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	# ************************************************************************************************************************************************
	## Rotor
	rotor = rootNode.addChild('rotor2')
	rotor.addObject('EulerImplicitSolver', name='odesolver')
	rotor.addObject('SparseLDLSolver', name='linearSolver', template="CompressedRowSparseMatrixMat3x3d")
	rotor.addObject('MechanicalObject', template='Rigid3d')
	rotor.addObject('UniformMass', totalMass='1e-10')
	rotor.addObject('LinearSolverConstraintCorrection')

	visual = rotor.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/rotor.obj', translation=[255, -1, 5], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,1.0]) # Color model 0-1 (RGB alpha)
	visual.addObject('RigidMapping')

	# ************************************************************************************************************************************************
	## Cavity
	cavity1 = finger.addChild('cavity1')
	cavity1.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_left.obj', translation = [115, 0, 0], rotation=[90, 0, 180])
	cavity1.addObject('MeshTopology', src='@loader', name='topo')
	cavity1.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity1.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity1.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	cavity2 = finger.addChild('cavity2')
	cavity2.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_right.obj', translation = [115, 0, 0], rotation=[90, 0, 180])
	cavity2.addObject('MeshTopology', src='@loader', name='topo')
	cavity2.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity2.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity2.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	visual = cavity1.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_left.obj', translation = [115, 0, 0], rotation=[90, 0, 180])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	visual = cavity2.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_right.obj', translation = [115, 0, 0], rotation=[90, 0, 180])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')


	# ************************************************************************************************************************************************
	## Spine
	spine = rootNode.addChild('spine2')
	spine.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness=0.0, rayleighMass=0.0)
	spine.addObject('SparseLDLSolver', name="LinearSolver")

	spine.addObject('MeshVTKLoader', name='loader', filename="mesh/arm_support.vtu", translation = [115, 0, 0], rotation=[90, 0, 180])
	spine.addObject('MechanicalObject', name='dofs', src = '@loader', template='Vec3d')
	spine.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	spine.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	spine.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	spine.addObject('UniformMass', src = '@topo', totalMass="15e-6")
	spine.addObject('TetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 75.5)
	spine.addObject('BoxROI', name='boxROI',box="85 -15 -22 95 16 22", drawBoxes = True)
	# spine.addObject('BoxROI', name='boxROI',box="-15 -95 -22 16 -85 22", drawBoxes = True)
	spine.addObject('FixedConstraint', indices = '@boxROI.indices')
	spine.addObject('LinearSolverConstraintCorrection')

	collision = spine.addChild('collision')
	collision.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [115, 0, 0], rotation=[90, 0, 180], triangulate=True)
	collision.addObject('MeshTopology', src='@loader')
	collision.addObject('MechanicalObject', template='Vec3d')
	collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	collision.addObject('BarycentricMapping')

	visual = spine.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [115, 0, 0], rotation=[90, 0, 180])
	visual.addObject('OglModel', src='@loader', color=[0.5,0,0,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	constraint_finger = finger.addChild('constraint_finger')
	constraint_finger.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'+'\n'+ '0 0 -5'+'\n'+'0 5 0'+'\n'+'0 -5 0'+ '\n' + 
		'-30 0 5'+'\n'+ '-30 0 -5'+'\n'+'-30 5 0'+'\n'+'-30 -5 0'+ '\n'+
		'-60 0 5'+'\n'+ '-60 0 -5'+'\n'+'-60 5 0'+'\n'+'-60 -5 0'+ '\n'+
		'-90 0 5'+'\n'+'-90 0 -5'+'\n'+'-90 5 0'+'\n'+'-90 -5 0'+ '\n'+
		'-120 0 5'+'\n'+'-120 0 -5'+'\n'+'-120 5 0'+'\n'+'-120 -5 0', translation = [115, 0, 0], rotation=[90, 0, 180])
	constraint_finger.addObject('BarycentricMapping')

	constraint_spine = spine.addChild('constraint_spine1')
	constraint_spine.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'+'\n'+ '0 0 -5'+'\n'+'0 5 0'+'\n'+'0 -5 0'+ '\n' + 
		'-30 0 5'+'\n'+ '-30 0 -5'+'\n'+ '-30 5 0'+'\n'+ '-30 -5 0'+ '\n'+
		'-60 0 5'+'\n'+ '-60 0 -5'+'\n'+ '-60 5 0'+'\n'+ '-60 -5 0'+ '\n'+
		'-90 0 5'+'\n'+ '-90 0 -5'+'\n'+ '-90 5 0'+'\n'+ '-90 -5 0'+ '\n'+
		'-120 0 5'+'\n'+'-120 0 -5'+'\n'+'-120 5 0'+'\n'+'-120 -5 0', translation = [115, 0, 0], rotation=[90, 0, 180])
	constraint_spine.addObject('BarycentricMapping')

	rootNode.addObject('BilateralInteractionConstraint', name = 'blc_finger2_spine2', template="Vec3d", object1 = "@finger2/constraint_finger/position", object2 = "@spine2/constraint_spine1/position", 
		first_point="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19", 
		second_point="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19")

	constraint_spine = spine.addChild('constraint_spine2')
	constraint_spine.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'-8 10 8'+'\n'+ '8 10 8'+'\n'+'-8 10 -2'+'\n'+'8 10 -2'+ '\n' + 
		'-8 0 8'+'\n'+ '8 0 8'+'\n'+'-8 0 -2'+'\n'+'8 0 -2'+ '\n' +
		'-8 -10 8'+'\n'+ '8 -10 8'+'\n'+'-8 -10 -2'+'\n'+'8 -10 -2', translation = [255, 0, 0], rotation=[0, 0, 90])
	constraint_spine.addObject('BarycentricMapping')

	constraint_rotor = rotor.addChild('constraint_rotor')
	constraint_rotor.addObject('MechanicalObject', name='position', position =
		'-8 10 8'+'\n'+ '8 10 8'+'\n'+'-8 10 -2'+'\n'+'8 10 -2'+ '\n' + 
		'-8 0 8'+'\n'+ '8 0 8'+'\n'+'-8 0 -2'+'\n'+'8 0 -2'+ '\n' +
		'-8 -10 8'+'\n'+ '8 -10 8'+'\n'+'-8 -10 -2'+'\n'+'8 -10 -2', translation = [255, 0, 0], rotation=[0, 0, 90])
	constraint_rotor.addObject('RigidMapping')

	rootNode.addObject('BilateralInteractionConstraint', name = 'blc_spine2_rotor2', template="Vec3d", object1 = "@spine2/constraint_spine2/position", object2 = "@rotor2/constraint_rotor/position", 
		first_point="0 1 2 3 4 5 6 7 8 9 10 11", 
		second_point="0 1 2 3 4 5 6 7 8 9 10 11")

	effector = spine.addChild('Effector')
	effectorMO = effector.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0,-150.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[115, 0, 0], rotation = [0, 0, 90])
	effector.addObject('BarycentricMapping')

	origin = spine.addChild('Origin')
	originMO = origin.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0, 30.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[115, 0, 0], rotation = [0, 0, 90])
	origin.addObject('BarycentricMapping')	
	spine.addObject(FingerController(node=rootNode, pos1= origin.dofs.position.value, pos2 = rootNode.getChild('spine2').Effector.dofs.position.value , idx=2))


	###


	# ************************************************************************************************************************************************
	## Finger
	finger = rootNode.addChild('finger3')
	finger.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness="0.0", rayleighMass="0.0")
	finger.addObject('SparseLDLSolver', name="LinearSolver")

	finger.addObject('MeshVTKLoader', name='loader', filename="mesh/SPA.vtu", translation = [0, 115, 0], rotation=[90, 0, 270])
	finger.addObject('MechanicalObject', name='dofs', template='Vec3d', src = '@loader')
	finger.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	finger.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	finger.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	finger.addObject('UniformMass', totalMass="115e-6", src = '@topo')
	#finger.addObject('TetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 0.6)
	mu1 = 0.24203
	lamb = 0
	finger.addObject('TetrahedronHyperelasticityFEMForceField', template='Vec3d', name='FEM', src ='@topo', ParameterSet=str(mu1)+' '+str(lamb),materialName="NeoHookean")
	finger.addObject('LinearSolverConstraintCorrection')

	collision = finger.addChild('collision')
	collision.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [0, 115, 0], rotation=[90, 0, 270], triangulate=True)
	collision.addObject('MeshTopology', src='@loader')
	collision.addObject('MechanicalObject', template='Vec3d')
	collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	collision.addObject('BarycentricMapping')

	visual = finger.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [0, 115, 0], rotation=[90, 0, 270])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,0.25]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	# ************************************************************************************************************************************************
	## Rotor
	rotor = rootNode.addChild('rotor3')
	rotor.addObject('EulerImplicitSolver', name='odesolver')
	rotor.addObject('SparseLDLSolver', name='linearSolver', template="CompressedRowSparseMatrixMat3x3d")
	rotor.addObject('MechanicalObject', template='Rigid3d')
	rotor.addObject('UniformMass', totalMass='1e-10')
	rotor.addObject('LinearSolverConstraintCorrection')

	visual = rotor.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/rotor.obj', translation=[1, 255, 5], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,1.0]) # Color model 0-1 (RGB alpha)
	visual.addObject('RigidMapping')

	# ************************************************************************************************************************************************
	## Cavity
	cavity1 = finger.addChild('cavity1')
	cavity1.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_left.obj', translation = [0, 115, 0], rotation=[90, 0, 270])
	cavity1.addObject('MeshTopology', src='@loader', name='topo')
	cavity1.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity1.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity1.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	cavity2 = finger.addChild('cavity2')
	cavity2.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_right.obj', translation = [0, 115, 0], rotation=[90, 0, 270])
	cavity2.addObject('MeshTopology', src='@loader', name='topo')
	cavity2.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity2.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity2.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	visual = cavity1.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_left.obj', translation = [0, 115, 0], rotation=[90, 0, 270])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	visual = cavity2.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_right.obj', translation = [0, 115, 0], rotation=[90, 0, 270])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	# ************************************************************************************************************************************************
	## Spine
	spine = rootNode.addChild('spine3')
	spine.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness=0.0, rayleighMass=0.0)
	spine.addObject('SparseLDLSolver', name="LinearSolver")

	spine.addObject('MeshVTKLoader', name='loader', filename="mesh/arm_support.vtu", translation = [0, 115, 0], rotation=[90, 0, 270])
	spine.addObject('MechanicalObject', name='dofs', src = '@loader', template='Vec3d')
	spine.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	spine.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	spine.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	spine.addObject('UniformMass', src = '@topo', totalMass="15e-6")
	spine.addObject('TetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 75.5)
	# spine.addObject('BoxROI', name='boxROI',box="1 20 -22 30 30 22", drawBoxes = True)
	spine.addObject('BoxROI', name='boxROI',box="-15 85 -22 16 95 22", drawBoxes = True)
	spine.addObject('FixedConstraint', indices = '@boxROI.indices')
	spine.addObject('LinearSolverConstraintCorrection')

	collision = spine.addChild('collision')
	collision.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [0, 115, 0], rotation=[90, 0, 270], triangulate=True)
	collision.addObject('MeshTopology', src='@loader')
	collision.addObject('MechanicalObject', template='Vec3d')
	collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	collision.addObject('BarycentricMapping')

	visual = spine.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [0, 115, 0], rotation=[90, 0, 270])
	visual.addObject('OglModel', src='@loader', color=[0.5,0,0,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	constraint_finger = finger.addChild('constraint_finger')
	constraint_finger.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'+'\n'+ '0 0 -5'+'\n'+'0 5 0'+'\n'+'0 -5 0'+ '\n' + 
		'-30 0 5'+'\n'+ '-30 0 -5'+'\n'+'-30 5 0'+'\n'+'-30 -5 0'+ '\n'+
		'-60 0 5'+'\n'+ '-60 0 -5'+'\n'+'-60 5 0'+'\n'+'-60 -5 0'+ '\n'+
		'-90 0 5'+'\n'+'-90 0 -5'+'\n'+'-90 5 0'+'\n'+'-90 -5 0'+ '\n'+
		'-120 0 5'+'\n'+'-120 0 -5'+'\n'+'-120 5 0'+'\n'+'-120 -5 0', translation = [0, 115, 0], rotation=[90, 0, 270])
	constraint_finger.addObject('BarycentricMapping')

	constraint_spine = spine.addChild('constraint_spine1')
	constraint_spine.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'+'\n'+ '0 0 -5'+'\n'+'0 5 0'+'\n'+'0 -5 0'+ '\n' + 
		'-30 0 5'+'\n'+ '-30 0 -5'+'\n'+ '-30 5 0'+'\n'+ '-30 -5 0'+ '\n'+
		'-60 0 5'+'\n'+ '-60 0 -5'+'\n'+ '-60 5 0'+'\n'+ '-60 -5 0'+ '\n'+
		'-90 0 5'+'\n'+ '-90 0 -5'+'\n'+ '-90 5 0'+'\n'+ '-90 -5 0'+ '\n'+
		'-120 0 5'+'\n'+'-120 0 -5'+'\n'+'-120 5 0'+'\n'+'-120 -5 0', translation = [0, 115, 0], rotation=[90, 0, 270])
	constraint_spine.addObject('BarycentricMapping')

	rootNode.addObject('BilateralInteractionConstraint', name = 'blc_finger3_spine3', template="Vec3d", object1 = "@finger3/constraint_finger/position", object2 = "@spine3/constraint_spine1/position", 
		first_point="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19", 
		second_point="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19")

	constraint_spine = spine.addChild('constraint_spine2')
	constraint_spine.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'-8 10 8'+'\n'+ '8 10 8'+'\n'+'-8 10 -2'+'\n'+'8 10 -2'+ '\n' + 
		'-8 0 8'+'\n'+ '8 0 8'+'\n'+'-8 0 -2'+'\n'+'8 0 -2'+ '\n' +
		'-8 -10 8'+'\n'+ '8 -10 8'+'\n'+'-8 -10 -2'+'\n'+'8 -10 -2', translation = [0, 255, 0], rotation=[0, 0, 180])
	constraint_spine.addObject('BarycentricMapping')

	constraint_rotor = rotor.addChild('constraint_rotor')
	constraint_rotor.addObject('MechanicalObject', name='position', position =
		'-8 10 8'+'\n'+ '8 10 8'+'\n'+'-8 10 -2'+'\n'+'8 10 -2'+ '\n' + 
		'-8 0 8'+'\n'+ '8 0 8'+'\n'+'-8 0 -2'+'\n'+'8 0 -2'+ '\n' +
		'-8 -10 8'+'\n'+ '8 -10 8'+'\n'+'-8 -10 -2'+'\n'+'8 -10 -2', translation = [0, 255, 0], rotation=[0, 0, 180])
	constraint_rotor.addObject('RigidMapping')

	rootNode.addObject('BilateralInteractionConstraint', name = 'blc_spine3_rotor3', template="Vec3d", object1 = "@spine3/constraint_spine2/position", object2 = "@rotor3/constraint_rotor/position", 
		first_point="0 1 2 3 4 5 6 7 8 9 10 11", 
		second_point="0 1 2 3 4 5 6 7 8 9 10 11")

	effector = spine.addChild('Effector')
	effectorMO = effector.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0,-150.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[0, 115, 0], rotation = [0, 0, 180])
	effector.addObject('BarycentricMapping')

	origin = spine.addChild('Origin')
	originMO = origin.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0, 30.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[0, 115, 0], rotation = [0, 0, 180])
	origin.addObject('BarycentricMapping')	
	spine.addObject(FingerController(node=rootNode, pos1= origin.dofs.position.value, pos2 = rootNode.getChild('spine3').Effector.dofs.position.value , idx=3))


	###


	# ************************************************************************************************************************************************
	## Finger
	finger = rootNode.addChild('finger4')
	finger.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness="0.0", rayleighMass="0.0")
	finger.addObject('SparseLDLSolver', name="LinearSolver")

	finger.addObject('MeshVTKLoader', name='loader', filename="mesh/SPA.vtu", translation = [-115, 0, 0], rotation=[90, 0, 0])
	finger.addObject('MechanicalObject', name='dofs', template='Vec3d', src = '@loader')
	finger.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	finger.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	finger.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	finger.addObject('UniformMass', totalMass="115e-6", src = '@topo')
	#finger.addObject('TetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 0.6)
	mu1 = 0.24203
	lamb = 0
	finger.addObject('TetrahedronHyperelasticityFEMForceField', template='Vec3d', name='FEM', src ='@topo', ParameterSet=str(mu1)+' '+str(lamb),materialName="NeoHookean")
	finger.addObject('LinearSolverConstraintCorrection')

	collision = finger.addChild('collision')
	collision.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [-115, 0, 0], rotation=[90, 0, 0], triangulate=True)
	collision.addObject('MeshTopology', src='@loader')
	collision.addObject('MechanicalObject', template='Vec3d')
	collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	collision.addObject('BarycentricMapping')

	visual = finger.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/SPA.obj', translation = [-115, 0, 0], rotation=[90, 0, 0])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,0.25]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	# ************************************************************************************************************************************************
	## Rotor
	rotor = rootNode.addChild('rotor4')
	rotor.addObject('EulerImplicitSolver', name='odesolver')
	rotor.addObject('SparseLDLSolver', name='linearSolver', template="CompressedRowSparseMatrixMat3x3d")
	rotor.addObject('MechanicalObject', template='Rigid3d')
	rotor.addObject('UniformMass', totalMass='1e-10')
	rotor.addObject('LinearSolverConstraintCorrection')

	visual = rotor.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/rotor.obj', translation=[-255, 1, 5], rotation=[90, 0, 90])
	visual.addObject('OglModel', src='@loader', color=[0.61,0.61,0.61,1.0]) # Color model 0-1 (RGB alpha)
	visual.addObject('RigidMapping')

	# ************************************************************************************************************************************************
	## Cavity
	cavity1 = finger.addChild('cavity1')
	cavity1.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_left.obj', translation = [-115, 0, 0], rotation=[90, 0, 0])
	cavity1.addObject('MeshTopology', src='@loader', name='topo')
	cavity1.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity1.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity1.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	cavity2 = finger.addChild('cavity2')
	cavity2.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_right.obj', translation = [-115, 0, 0], rotation=[90, 0, 0])
	cavity2.addObject('MeshTopology', src='@loader', name='topo')
	cavity2.addObject('MechanicalObject', name='dofs', template='Vec3d')
	cavity2.addObject('SurfacePressureConstraint', name='SurfaceForceField', template='Vec3d', value = 0.0, triangles='@topo.triangles', valueType='pressure')
	cavity2.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=False)

	visual = cavity1.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_left.obj', translation = [-115, 0, 0], rotation=[90, 0, 0])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	visual = cavity2.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cavity_right.obj', translation = [-115, 0, 0], rotation=[90, 0, 0])
	visual.addObject('OglModel', src='@loader', color=[0,0,0.5,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	# ************************************************************************************************************************************************
	## Spine
	spine = rootNode.addChild('spine4')
	spine.addObject('EulerImplicitSolver', name="Solver", rayleighStiffness=0.0, rayleighMass=0.0)
	spine.addObject('SparseLDLSolver', name="LinearSolver")

	spine.addObject('MeshVTKLoader', name='loader', filename="mesh/arm_support.vtu", translation = [-115, 0, 0], rotation=[90, 0, 0])
	spine.addObject('MechanicalObject', name='dofs', src = '@loader', template='Vec3d')
	spine.addObject('TetrahedronSetTopologyContainer', name="topo", src ='@loader')
	spine.addObject('TetrahedronSetTopologyModifier' ,  name="Modifier")
	spine.addObject('TetrahedronSetGeometryAlgorithms',name="GeomAlgo")
	spine.addObject('UniformMass', src = '@topo', totalMass="15e-6")
	spine.addObject('TetrahedronFEMForceField', name='FEM', src ='@topo', poissonRatio = 0.45, youngModulus = 75.5)
	spine.addObject('BoxROI', name='boxROI',box="-95 -15 -22 -85 16 22", drawBoxes = True)
	# spine.addObject('BoxROI', name='boxROI',box="-15 -95 -22 16 -85 22", drawBoxes = True)
	spine.addObject('FixedConstraint', indices = '@boxROI.indices')
	spine.addObject('LinearSolverConstraintCorrection')

	collision = spine.addChild('collision')
	collision.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [-115, 0, 0], rotation=[90, 0, 0], triangulate=True)
	collision.addObject('MeshTopology', src='@loader')
	collision.addObject('MechanicalObject', template='Vec3d')
	collision.addObject('TriangleCollisionModel', selfCollision=False, group='1')
	collision.addObject('LineCollisionModel', selfCollision=False, group='1')
	collision.addObject('PointCollisionModel', selfCollision=False, group='1')
	collision.addObject('BarycentricMapping')

	visual = spine.addChild('visual')
	visual.addObject('MeshOBJLoader', name='loader', filename='mesh/arm_support.obj', translation = [-115, 0, 0], rotation=[90, 0, 0])
	visual.addObject('OglModel', src='@loader', color=[0.5,0,0,0.5]) # Color model 0-1 (RGB alpha)
	visual.addObject('BarycentricMapping')

	constraint_finger = finger.addChild('constraint_finger')
	constraint_finger.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'+'\n'+ '0 0 -5'+'\n'+'0 5 0'+'\n'+'0 -5 0'+ '\n' + 
		'-30 0 5'+'\n'+ '-30 0 -5'+'\n'+'-30 5 0'+'\n'+'-30 -5 0'+ '\n'+
		'-60 0 5'+'\n'+ '-60 0 -5'+'\n'+'-60 5 0'+'\n'+'-60 -5 0'+ '\n'+
		'-90 0 5'+'\n'+'-90 0 -5'+'\n'+'-90 5 0'+'\n'+'-90 -5 0'+ '\n'+
		'-120 0 5'+'\n'+'-120 0 -5'+'\n'+'-120 5 0'+'\n'+'-120 -5 0', translation = [-115, 0, 0], rotation=[90, 0, 0])
	constraint_finger.addObject('BarycentricMapping')

	constraint_spine = spine.addChild('constraint_spine1')
	constraint_spine.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'0 0 5'+'\n'+ '0 0 -5'+'\n'+'0 5 0'+'\n'+'0 -5 0'+ '\n' + 
		'-30 0 5'+'\n'+ '-30 0 -5'+'\n'+ '-30 5 0'+'\n'+ '-30 -5 0'+ '\n'+
		'-60 0 5'+'\n'+ '-60 0 -5'+'\n'+ '-60 5 0'+'\n'+ '-60 -5 0'+ '\n'+
		'-90 0 5'+'\n'+ '-90 0 -5'+'\n'+ '-90 5 0'+'\n'+ '-90 -5 0'+ '\n'+
		'-120 0 5'+'\n'+'-120 0 -5'+'\n'+'-120 5 0'+'\n'+'-120 -5 0', translation = [-115, 0, 0], rotation=[90, 0, 0])
	constraint_spine.addObject('BarycentricMapping')

	rootNode.addObject('BilateralInteractionConstraint', name = 'blc_finger4_spine4', template="Vec3d", object1 = "@finger4/constraint_finger/position", object2 = "@spine4/constraint_spine1/position", 
		first_point="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19", 
		second_point="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19")

	constraint_spine = spine.addChild('constraint_spine2')
	constraint_spine.addObject('MechanicalObject', name='position', template = 'Vec3d', position =
		'-8 10 8'+'\n'+ '8 10 8'+'\n'+'-8 10 -2'+'\n'+'8 10 -2'+ '\n' + 
		'-8 0 8'+'\n'+ '8 0 8'+'\n'+'-8 0 -2'+'\n'+'8 0 -2'+ '\n' +
		'-8 -10 8'+'\n'+ '8 -10 8'+'\n'+'-8 -10 -2'+'\n'+'8 -10 -2', translation = [-255, 0, 0], rotation=[0, 0, 270])
	constraint_spine.addObject('BarycentricMapping')

	constraint_rotor = rotor.addChild('constraint_rotor')
	constraint_rotor.addObject('MechanicalObject', name='position', position =
		'-8 10 8'+'\n'+ '8 10 8'+'\n'+'-8 10 -2'+'\n'+'8 10 -2'+ '\n' + 
		'-8 0 8'+'\n'+ '8 0 8'+'\n'+'-8 0 -2'+'\n'+'8 0 -2'+ '\n' +
		'-8 -10 8'+'\n'+ '8 -10 8'+'\n'+'-8 -10 -2'+'\n'+'8 -10 -2', translation = [-255, 0, 0], rotation=[0, 0, 270])
	constraint_rotor.addObject('RigidMapping')

	rootNode.addObject('BilateralInteractionConstraint', name = 'blc_spine3_rotor3', template="Vec3d", object1 = "@spine4/constraint_spine2/position", object2 = "@rotor4/constraint_rotor/position", 
		first_point="0 1 2 3 4 5 6 7 8 9 10 11", 
		second_point="0 1 2 3 4 5 6 7 8 9 10 11")

	effector = spine.addChild('Effector')
	effectorMO = effector.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0,-150.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[-115, 0, 0], rotation = [0, 0, 270])
	effector.addObject('BarycentricMapping')

	origin = spine.addChild('Origin')
	originMO = origin.addObject('MechanicalObject', name='dofs', template='Vec3',position=[0.0, 30.0, 0.0], showObject=True, showObjectScale=5, drawMode=2, showColor='blue', translation=[-115, 0, 0], rotation = [0, 0, 270])
	origin.addObject('BarycentricMapping')	
	spine.addObject(FingerController(node=rootNode, pos1= origin.dofs.position.value, pos2 = rootNode.getChild('spine4').Effector.dofs.position.value , idx=4))

	return rootNode
