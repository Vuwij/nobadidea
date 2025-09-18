import math
import pybullet as pb

class Robot:
    def __init__(self, start_pos, start_yaw=0):
        stl_file = "penguin_on_the_skateboard.stl"

        # --- Base visual ---
        self.visual_shape_id = pb.createVisualShape(
            shapeType=pb.GEOM_MESH,
            fileName=stl_file,
            meshScale=[0.5, 0.5, 0.5],
            rgbaColor=[0.1, 0.1, 0.1, 1]
        )

        # --- Base collision ---
        radius = 0.05
        height = 0.3
        self.collision_shape_id = pb.createCollisionShape(
            shapeType=pb.GEOM_CYLINDER,
            radius=radius,
            height=height,
            collisionFramePosition=[0, 0, height / 2]
        )

        # --- Arm shape (cylinder) ---
        arm_radius = 0.02
        arm_length = 0.2
        self.arm_collision = pb.createCollisionShape(
            shapeType=pb.GEOM_CYLINDER,
            radius=0.0001,
            height=0.0001
        )
        self.arm_visual = pb.createVisualShape(
            shapeType=pb.GEOM_CYLINDER,
            radius=arm_radius,
            length=arm_length,
            rgbaColor=[0.8, 0.2, 0.2, 1],
            visualFramePosition=[0, 0, arm_length / 2]  # base at joint
        )

        # Arm pivot offset: start from robot center, extend forward (north = +y)
        arm_offset = [-radius, 0, height * 0.75]

        # Build multibody with one link (the arm)
        self.body_id = pb.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=self.collision_shape_id,
            baseVisualShapeIndex=self.visual_shape_id,
            basePosition=start_pos,
            baseOrientation=pb.getQuaternionFromEuler([0, 0, start_yaw]),
            linkMasses=[0.1],
            linkCollisionShapeIndices=[self.arm_collision],
            linkVisualShapeIndices=[self.arm_visual],
            linkPositions=[arm_offset],
            # Cylinder axis along local +Z, rotate to horizontal pointing west
            linkOrientations=[pb.getQuaternionFromEuler([0, -math.pi / 2, 0])],
            linkInertialFramePositions=[[0, 0, 0]],
            linkInertialFrameOrientations=[[0, 0, 0, 1]],
            linkParentIndices=[0],
            linkJointTypes=[pb.JOINT_REVOLUTE],
            linkJointAxis=[[0, 1, 0]]  # rotate around Y for up/down tilt
        )

        self.arm_joint_index = 0  # only one link

    def spin(self, speed=0.5):
        pos, orn = pb.getBasePositionAndOrientation(self.body_id)
        euler = pb.getEulerFromQuaternion(orn)
        yaw = euler[2] + speed * (1./240.)
        new_orn = pb.getQuaternionFromEuler([0, 0, yaw])
        pb.resetBasePositionAndOrientation(self.body_id, pos, new_orn)

    def move_forward(self, speed=1.0):
        pos, orn = pb.getBasePositionAndOrientation(self.body_id)
        euler = pb.getEulerFromQuaternion(orn)
        yaw = euler[2]
        dx = speed * (1./240.) * math.cos(yaw)
        dy = speed * (1./240.) * math.sin(yaw)
        new_pos = [pos[0] + dx, pos[1] + dy, pos[2]]
        pb.resetBasePositionAndOrientation(self.body_id, new_pos, orn)

    def point_to_floor(self, floor: int):
        """Rotate arm to point to a floor:
           floor 0 = 0°, floor 1 = 45°, floor 2 = 60°"""
        mapping = {0: 0, 1: math.radians(60), 2: math.radians(75)}
        target_angle = mapping.get(floor, 0)

        pb.setJointMotorControl2(
            bodyUniqueId=self.body_id,
            jointIndex=self.arm_joint_index,
            controlMode=pb.POSITION_CONTROL,
            targetPosition=target_angle,
            force=10
        )
