from klampt import *
from klampt.model.create import *
from klampt.model import ik
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from klampt.math import vectorops, so3, se3
from openkukagrip import openhand
from OpenGL.GL import *
import time
import numpy as np
from collision_color import *
import itertools
from MeshGeometry import FaceGeom


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
        self.angles = None
        self.current_trimesh = None
        self.faces_geom = []
        self.world_point1s = []
        self.world_point2s = []

    def init_local_pos(self):
        # self.local_pos.append((0.01, 0.01, 0.15))
        # self.local_pos.append((-0.01, 0.01, 0.15))
        # self.local_pos.append((0, -0.01, 0.15))

        # self.local_pos.append((0.01, 0.01, 0.2))
        # self.local_pos.append((-0.01, 0.01, 0.2))
        # self.local_pos.append((0, -0.01, 0.2))

        self.local_pos.append((0.01, 0.01, 0.15))
        self.local_pos.append((-0.01, 0.01, 0.15))
        self.local_pos.append((0, -0.01, 0.15))
        return

    def init_world_pos(self):
        self.world_pos.append(vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0]))
        self.world_pos.append(vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0]))
        self.world_pos.append(vectorops.add(self.object.getTransform()[1], [0, -0.01, 0]))
        self.world_pos.append(vectorops.add(self.object.getTransform()[1], [0, 0, 0.2]))
        return

    def open(self, q, amount=1.0):
        """Computes a configuration identical to q but with hand open
        if amount = 1 or closed if amount = 0"""
        return openhand(q, self.hand, amount)

    def ikSolver(self, robot, cspace, obj_pt):
        """Returns an IK solver that places the hand center at obj_pt with
        the palm oriented along obj_axis"""
        solution = self.goal_transform_transit(robot, cspace, obj_pt)

        if solution is not None and solution is not False:
            grasp, grasparm = solution
            return grasp, grasparm
        else:
            return solution

    def goal_transform_transit(self, robot, cspace, obj_index):
        for n1, n2 in itertools.combinations(self.faces_geom, 2):
            if (self.faces_geom.index(n1) == 10 and self.faces_geom.index(n2) == 17) or (self.faces_geom.index(n2) == 10 and self.faces_geom.index(n1) == 17):
                start = time.time()
                print("VECTORS ARE", n1.normal_vec, n2.normal_vec)
                cross_product = np.cross(n1.normal_vec, n2.normal_vec)
                print("CROSS PRODUCT", cross_product)
                cross_norm = np.linalg.norm(cross_product)
                if isclose(cross_norm, 0, abs_tol=1e-3):
                    # set the goal to match local points on the arm with the object points
                    self.create_world_pos(n1.face_center, n2.face_center)
                    finger1 = ik.objective(robot.link("grip2"),
                                        local=[self.GripperPoint1s[0], self.GripperPoint1s[1], self.GripperPoint1s[2]],
                                        world=[self.world_point1s[1], self.world_point1s[0], self.world_point1s[2]])
                    finger2 = ik.objective(robot.link("grip4"),
                                        local=[self.GripperPoint2s[0], self.GripperPoint2s[1], self.GripperPoint2s[2]],
                                        world=[self.world_point2s[1], self.world_point2s[0], self.world_point2s[2]])
                    res = self.solve(robot, cspace, obj_index,  finger1, finger2)
                    if res is not None:
                        return res
                    else:
                        print("Couldn't find grasp")
                end = time.time()
                print("iteration time", end - start)
        print("Solution to the transit problem hasn't been found")
        return False

    def solve(self, robot, cspace, obj_index, f1, f2):
        """Set up the IKSolver and check the feasibility of the found salvation"""
        grasp = None
        solver = IKSolver(robot)
        solver.add(f1)
        solver.add(f2)
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

    def change_the_goal(self, obj_index):
        self.object = world.rigidObject(obj_index)
        return 0

    def normal_vector(self):
        vector1 = vectorops.sub(self.world_pos[1], self.world_pos[0])
        vector2 = vectorops.sub(self.world_pos[3], self.world_pos[0])
        normal = vectorops.cross(vector1, vector2)
        return normal

    def force_closure(self, contact_points):
        return self.object.contact.ForceClosure(contact_points)

    def create_world_pos(self, face_cntr1, face_cntr2):
        self.world_point1s = []
        self.world_point2s = []
        self.world_point1s.append(vectorops.add(face_cntr1, [0.01, 0, 0]))
        self.world_point1s.append(vectorops.add(face_cntr1, [-0.01, 0, 0]))
        self.world_point1s.append(vectorops.add(face_cntr1, [0, 0, -0.01]))

        self.world_point2s.append(vectorops.add(face_cntr2, [0.01, 0, 0]))
        self.world_point2s.append(vectorops.add(face_cntr2, [-0.01, 0, 0]))
        self.world_point2s.append(vectorops.add(face_cntr2, [0, 0, -0.01]))


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)
