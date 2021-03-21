from klampt import *
from klampt.model.create import *
from klampt.model import ik
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from klampt.math import vectorops, so3, se3
from openkukagrip import openhand
from OpenGL.GL import *
import time
import math
import numpy as np
from collision_color import *
import itertools
from MeshGeometry import FaceGeom
from TransformFunc import *


goal_rotation_transfer = None
goal_config_transfer = None
start_angle = None

class Hand:
    """Defines basic elements of a hand and its arm.
    Members include link (the hand link index),
    localPosition1, localPosition2, localPosition3 (three points that define rotations of the hand),
    armIndices (the hand's arm link indices),
    GripperPoint1 and GripperPoint2 (two points, describing the contact with the object.
    grip_config, which is start gripper configuration
    """

    def __init__(self, world, hand='kuka'):
        self.hand = hand
        self.armIndices = range(6, 13)
        self.handIndices = range(14, 19)
        self.gripperIndex = 14
        self.link = 14
        self.world = world
        #self.object = world.rigidObject(0)
        self.object = None
        self.local_pos = []
        self.init_local_pos()
        self.worldPositionRadius = 0.03
        self.GripperPoint1s = [vectorops.add([0.0, 0.043, 0.1275], [0.01, 0, 0]),
                               vectorops.add([0.0, 0.043, 0.1275], [-0.01, 0, 0]),
                               vectorops.add([0.0, 0.043, 0.1275], [0, 0, 0.01])]
        self.GripperPoint2s = [vectorops.add([0.0, -0.043, 0.1275], [0.01, 0, 0]),
                               vectorops.add([0.0, -0.043, 0.1275], [-0.01, 0, 0]),
                               vectorops.add([0.0, -0.043, 0.1275], [0, 0, 0.01])]
        self.world_pos = []
        self.inside_mesh = [0, 0, 0]
        self.current_trimesh = None
        self.faces_geom = []
        self.world_point1s = []
        self.world_point2s = []
        self.world_points = []

    def init_local_pos(self):
        self.local_pos.append((0.01, 0.01, 0.15))
        self.local_pos.append((-0.01, 0.01, 0.15))
        self.local_pos.append((0, -0.01, 0.15))
        return

    def init_world_points(self):
        self.world_points.append(vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0]))
        self.world_points.append(vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0]))
        self.world_points.append(vectorops.add(self.object.getTransform()[1], [0, -0.01, 0]))
        #self.world_pos.append(vectorops.add(self.object.getTransform()[1], [0, 0, 0.2]))
        return

    def open(self, q, amount=1.0):
        """Computes a configuration identical to q but with hand open
        if amount = 1 or closed if amount = 0"""
        return openhand(q, self.hand, amount)

    # def ikSolver(self, robot, cspace, obj_pt):
    #     """Returns an IK solver that places the hand center at obj_pt with
    #     the palm oriented along obj_axis"""
    #     solution = self.goal_transform_transit(robot, cspace, obj_pt)
    #
    #     if solution is not None and solution is not False:
    #         grasp, grasparm = solution
    #         return grasp, grasparm
    #     else:
    #         return solution

    def ikSolver(self, robot, cspace, obj_pt, transfer, rot):
        """Returns an IK solver that places the hand center at obj_pt with
        the palm oriented along obj_axis"""

        if not transfer:
            solution = self.goal_transform_transit(robot, cspace)
        else:
            solution = self.goal_rotation_transfer(robot, cspace, obj_pt, rot)

        if solution is not None:
            grasp, grasparm = solution
            return grasp, grasparm
        else:
            return None

    def goal_rotation_transfer(self, robot, cspace, obj_pt, rot):
        """
        Rotates the gripper and tries ungrasping from each side specified by 3 angles
        :param robot: Robot model used
        :param cspace: CSpace object with defined feasibility
        :param obj_pt: Coordinates of the object (point in the world)
        :param n: Used to set the step of each rotation. I.e: n = 3 => each step is pi/3
        :return: returns the configurations grasp and grasparm,  or None
        """
        global goal_rotation_transfer
        global start_angle

        i_old, j_old, k_old = 0, 0, 0

        if goal_rotation_transfer is not None:
            i_old = goal_rotation_transfer[0]
            j_old = goal_rotation_transfer[1]
            k_old = goal_rotation_transfer[2]

        start_angle = [0, 0, 0]

        world_position_matrix = [self.world_points[0], self.world_points[1], self.world_points[2]]

        # iterate through the best angles to put the object
        for i in range(i_old, 4):
            angleX = (i * math.pi/2, 0, 0)
            #angleX = ((i * math.pi / 2) + start_angle[0], 0, 0)
            for j in range(j_old, 4):
                angleY = (0, j * math.pi / 2, 0)
                #angleY = (0, (j * math.pi / 2) + start_angle[1], 0)
                for k in range(k_old, 4):
                    angleZ = (0, 0, k * math.pi / 2)
                    #angleZ = (0, 0, (k * math.pi / 2) + start_angle[2])
                    if rot:
                        if (i == 0 and j == 0 and k == 0) or (i == 2 and j == 2 and k == 2):
                            continue

                    # creates rotation matrix for specified angle
                    rot_matrixX = euler_angle_to_rotation(angleX)
                    rot_matrixY = euler_angle_to_rotation(angleY)
                    rot_matrixZ = euler_angle_to_rotation(angleZ)
                    # rotates the object in three directions
                    rotObjX = [so3.apply(rot_matrixX, vectorops.sub(world_position_matrix[x],
                                                                    self.object.getTransform()[1]))
                               for x in range(3)]
                    rotObjY = [so3.apply(rot_matrixY, rotObjX[h]) for h in range(3)]
                    rotObj_final = [so3.apply(rot_matrixZ, rotObjY[u]) for u in range(3)]

                    # stores the position of the object in the world's coordinate system
                    self.worldPosition1 = vectorops.add(obj_pt, rotObj_final[0])
                    self.worldPosition2 = vectorops.add(obj_pt, rotObj_final[1])
                    self.worldPosition3 = vectorops.add(obj_pt, rotObj_final[2])

                    # set the goal to match local points on the arm with the object points
                    goal = ik.objective(robot.link("tool0"),
                                        local=[self.local_pos[0], self.local_pos[1], self.local_pos[2]],
                                        world=[self.worldPosition2, self.worldPosition1, self.worldPosition3])
                    res = self.solve(robot, cspace, goal)
                    if res is not None:
                        k += 1
                        if k >= 4:
                            j += 1
                            k = 0
                            if j >= 4:
                                i += 1
                                j = 0
                        goal_rotation_transfer = [i, j, k]
                        return res
        print("Solution to the transfer problem hasn't been found")
        return None

    def goal_transform_transit(self, robot, cspace):

        for n1, n2 in itertools.combinations(self.faces_geom, 2):
            start = time.time()
            print("VECTORS ARE", n1.normal_vec, n2.normal_vec)
            cross_product = np.cross(n1.normal_vec, n2.normal_vec)
            print("CROSS PRODUCT", cross_product)
            cross_norm = np.linalg.norm(cross_product)
            if isclose(cross_norm, 0, abs_tol=1e-3):
                # set the goal to match local points on the arm with the object points
                center_of_mass = np.multiply(np.add(n1.face_center, n2.face_center), 0.5)
                self.create_world_pos(center_of_mass)
                goal = ik.objective(robot.link("tool0"),
                                    local=[self.local_pos[0], self.local_pos[1], self.local_pos[2]],
                                    world=[self.world_points[1], self.world_points[0], self.world_points[2]])
                res = self.solve(robot, cspace, goal)
                if res is not None:
                    return res
                else:
                    print("Couldn't find grasp")
            end = time.time()
            print("iteration time", end - start)
        print("Solution to the transit problem hasn't been found")
        return False

    def solve(self, robot, cspace, goal):
        """Set up the IKSolver and check the feasibility of the found salvation"""
        grasp = None
        solver = IKSolver(robot)
        solver.add(goal)
        solver.setActiveDofs(self.armIndices)
        numRestarts = 100
        for i in range(numRestarts):
            solver.sampleInitial()
            solver.setMaxIters(200)
            solver.setTolerance(1e-3)
            res = solver.solve()
            if res:
                grasp = robot.getConfig()
                grasparm = [grasp[i] for i in self.armIndices]
                if not cspace.feasible(grasparm):
                    print("Grasp config infeasible")
                    pass
                else:
                    print("FOUND IT GOTCHA")
                    return grasp, grasparm
            if grasp is None:
                # print("Grasp solve failed")
                pass

    def refresh_world_coord(self):
        self.world_pos[0] = vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0])
        self.world_pos[1] = vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0])
        self.world_pos[2] = vectorops.add(self.object.getTransform()[1], [0, -0.01, 0])
        self.world_pos[3] = vectorops.add(self.object.getTransform()[1], [0, 0, 0.2])
        return 0

    def refresh_world_points(self):
        self.world_points[0] = vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0])
        self.world_points[1] = vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0])
        self.world_points[2] = vectorops.add(self.object.getTransform()[1], [0, -0.01, 0])
        #self.world_pos[3] = vectorops.add(self.object.getTransform()[1], [0, 0, 0.2])
        return 0

    def change_the_goal(self, obj_index):
        self.object = self.world.rigidObject(obj_index)
        return 0

    def normal_vector(self):
        vector1 = vectorops.sub(self.world_pos[1], self.world_pos[0])
        vector2 = vectorops.sub(self.world_pos[3], self.world_pos[0])
        normal = vectorops.cross(vector1, vector2)
        return normal

    def force_closure(self, contact_points):
        return self.object.contact.ForceClosure(contact_points)

    def create_world_pos(self, face_cntr1):
        self.world_points = []
        self.world_points.append(vectorops.add(face_cntr1, [0.01, 0.01, 0]))
        self.world_points.append(vectorops.add(face_cntr1, [-0.01, 0.01, 0]))
        self.world_points.append(vectorops.add(face_cntr1, [0, -0.01, 0]))



def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)
