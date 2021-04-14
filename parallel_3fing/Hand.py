from klampt import *
from klampt.model.create import *
from klampt.model import ik
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from klampt.math import vectorops, so3, se3
from utilities.open3fing_parallel import openhand
from OpenGL.GL import *
import time
import math
import numpy as np
from utilities.collision_color import *
import itertools
from utilities.MeshGeometry import FaceGeom
from utilities.TransformFunc import *
import heapq

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
        self.gripper = "3fing_parallel"
        self.fingerLink1 = 15
        self.fingerLink2 = 16
        self.fingerLink3 = 17

        self.armIndices = range(6, 13)
        self.handIndices = range(15, 18)
        self.gripperIndex = 14
        self.link = 14
        self.world = world
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
        self.transit_points = []
        self.transfer_points = []
        self.grasped_face1 = None
        self.grasped_face2 = None
        self.heap = []
        self.start_angle = []
        self.indeces = [0, 0, 0]

    def init_local_pos(self):
        self.local_pos.append((0.01, 0.01, 0.15))
        self.local_pos.append((-0.01, 0.01, 0.15))
        self.local_pos.append((0, -0.01, 0.15))

        # self.local_pos.append((0, 0.01, 0.15))
        # self.local_pos.append((0, -0.01, 0.15))
        self.local_pos.append((0.01, 0.15, 0))
        self.local_pos.append((-0.01, 0.15, 0))
        return

    def init_transit_points(self):
        # self.world_points.append(vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0]))
        # self.world_points.append(vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0]))
        # self.world_points.append(vectorops.add(self.object.getTransform()[1], [0, -0.01, 0]))
        self.transit_points.append(vectorops.add(self.object.getTransform()[1], [0.01, 0, 0]))
        self.transit_points.append(vectorops.add(self.object.getTransform()[1], [-0.01, 0, 0]))
        return

    def init_transfer_points(self):
        self.transfer_points.append(vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0]))
        self.transfer_points.append(vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0]))
        self.transfer_points.append(vectorops.add(self.object.getTransform()[1], [0, -0.01, 0]))
        # self.world_points.append(vectorops.add(self.object.getTransform()[1], [0, 0.01, 0]))
        # self.world_points.append(vectorops.add(self.object.getTransform()[1], [0, -0.01, 0]))
        return

    def open(self, q, amount=1.0):
        """Computes a configuration identical to q but with hand open
        if amount = 1 or closed if amount = 0"""
        return openhand(q, self.hand, amount)

    def open_free(self, q, amount=1.0):
        """Computes a configuration identical to q but with hand open
        if amount = 1 or closed if amount = 0"""
        return openhand(q, self.hand, amount)

    def ikSolver(self, robot, cspace, obj_pt, transfer, rot):
        """Returns an IK solver that places the hand center at obj_pt with
        the palm oriented along obj_axis"""

        if not transfer:
            solution = self.goal_transform_transit(robot, cspace)
        else:
            solution = self.goal_definition_transfer(robot, cspace, obj_pt)
            # solution = self.goal_rotation_transfer(robot, cspace, obj_pt, rot)

        if solution:
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

        i_old, j_old, k_old = 0, 0, 0

        if goal_rotation_transfer is not None:
            i_old = goal_rotation_transfer[0]
            j_old = goal_rotation_transfer[1]
            k_old = goal_rotation_transfer[2]


        world_position_matrix = [self.transfer_points[0], self.transfer_points[1], self.transfer_points[2]]

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

    def goal_definition_transfer(self, robot, cspace, obj_pt):
        Tobj0 = self.object.getTransform()
        Thand0 = robot.link(self.link).getTransform()
        t0 = vectorops.sub(Thand0[1], Tobj0[1])
        for i in range(self.indeces[0], 4):
            for j in range(self.indeces[1], 4):
                for k in range(self.indeces[2], 4):
                    angle = (i * math.pi / 2, j * math.pi / 2, k * math.pi / 2)
                    rot_matrix = euler_angle_to_rotation(angle)
                    rotObj = so3.mul(rot_matrix, Thand0[0])
                    t = so3.apply(rot_matrix, t0)
                    goal = ik.objective(robot.link("gripper_body"),
                                        R=rotObj,
                                        t=vectorops.add(obj_pt, t))
                    res = self.solve(robot, cspace, goal)
                    if res is not None:
                        k += 1
                        if k >= 4:
                            j += 1
                            k = 0
                            if j >= 4:
                                i += 1
                                j = 0
                        self.indeces = [i, j, k]
                        return res
        print("Solution to the transfer problem hasn't been found")
        return None

    def goal_transform_transit(self, robot, cspace):
        start = time.time()
        # for n1, n2 in itertools.combinations(self.faces_geom, 2):
        #     print("VECTORS ARE", n1.normal_vec, n2.normal_vec)
        #     dot_product = np.dot(n1.normal_vec, n2.normal_vec)
        #     print("DOT_PRODUCT", dot_product)
        #     if dot_product >= 0:
        #         continue
        #     cross_product = np.cross(n1.normal_vec, n2.normal_vec)
        #     print("CROSS PRODUCT", cross_product)
        #     cross_norm = np.linalg.norm(cross_product)
        #     if isclose(cross_norm, 0, abs_tol=1e-3):
        #         # set the goal to match local points on the arm with the object points
        #         center_of_mass = np.multiply(np.add(n1.face_center, n2.face_center), 0.5)
        #         self.create_transit_pos(n1.face_center, n2.face_center, center_of_mass)
        #         goal = ik.objective(robot.link("tool0"),
        #                             local=[self.local_pos[3], self.local_pos[4]],
        #                             world=[self.transit_points[0], self.transit_points[1]])
        #         res = self.solve(robot, cspace, goal)
        #         if res is not None:
        #             self.grasped_face1 = n1.ind
        #             self.grasped_face2 = n2.ind
        #             return res
        #         else:
        #             print("Couldn't find grasp")
        cnt = 0
        while True:
            print("HEAP LENGTH", len(self.heap))
            try:
                center_of_mass, n1, n2 = heapq.heappop(self.heap)[1]
            except IndexError:
                break
            cnt += 1
            self.create_transit_pos(n1.face_center, n2.face_center, center_of_mass)
            goal = ik.objective(robot.link("gripper_body"),
                                local=[self.local_pos[3], self.local_pos[4]],
                                world=[self.transit_points[0], self.transit_points[1]])
            res = self.solve(robot, cspace, goal)
            if res is not None:
                end = time.time()
                self.grasped_face1 = n1.ind
                self.grasped_face2 = n2.ind
                print("Solved within first ", cnt, "iterations,", " soving time: ", start - end)
                return res
            else:
                print("Couldn't find grasp")

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

    def refresh_world_points(self):
        self.transfer_points[0] = vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0])
        self.transfer_points[1] = vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0])
        self.transfer_points[2] = vectorops.add(self.object.getTransform()[1], [0, -0.01, 0])

        # self.transit_points[0] = vectorops.add(self.object.getTransform()[1], [0, 0.01, 0])
        # self.transit_points[1] = vectorops.add(self.object.getTransform()[1], [0, -0.01, 0])
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

    def create_transit_pos(self, face_cntr1, face_cntr2, centroid):
        vec1 = face_cntr1 - centroid
        vec1_norm = np.linalg.norm(vec1)
        vec1_length = 0.01/vec1_norm
        vec1 = np.multiply(vec1, vec1_length)

        vec2 = face_cntr2 - centroid
        vec2_norm = np.linalg.norm(vec2)
        vec2_length = 0.01/vec2_norm
        vec2 = np.multiply(vec2, vec2_length)

        # self.transit_points[0] = vectorops.add(face_cntr1, [0, 0.01, 0])
        # self.transit_points[1] = vectorops.add(face_cntr1, [0, -0.01, 0])

        self.transit_points[0] = vectorops.add(centroid, vec1)
        self.transit_points[1] = vectorops.add(centroid, vec2)


