from klampt import *
from klampt.model import contact
from klampt.model.create import *
from klampt.model import ik
from klampt.model.collide import WorldCollider
from klampt.model.trajectory import RobotTrajectory
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from klampt import vis
from klampt.math import vectorops, so3, se3
from klampt.plan.cspace import CSpace, MotionPlan
from openkukagrip import openhand
from OpenGL.GL import *
import math
import time
import numpy as np
from numpy import asarray
from numpy import savetxt
from numpy import random
import trimesh
import networkx as nx
from collision_color import *

# global variable used to store the latest tried config of the arm in case path wasn't found
goal_config_transit = None
goal_rotation_transfer = None
save_directory = 'grasping_database/cube_grasp_uti4.csv'

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
        self.object = world.rigidObject(0)
        self.local_pos = []
        self.init_local_pos()
        # self.localPosition1 = (0.01, 0.01, 0.15)
        # self.localPosition2 = (-0.01, 0.01, 0.15)
        # self.localPosition3 = (0, -0.01, 0.15)
        self.worldPositionRadius = 0.03
        self.GripperPoint1 = (0.0, 0.043, 0.1275)
        self.GripperPoint2 = (0.0, -0.043, 0.1275)
        self.world_pos = []
        # self.worldPosition1 = vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0])
        # self.worldPosition2 = vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0])
        # self.worldPosition3 = vectorops.add(self.object.getTransform()[1], [0, -0.01, 0])
        # self.worldPosition4 = vectorops.add(self.object.getTransform()[1], [0, 0, 0.2])
        self.init_world_pos()
        self.inside_mesh = [0, 0, 0]
        self.angles = None
        # self.subrobot=model.subrobot.SubRobotModel(self.world.robot(0),["grip0", "grip1", "grip2", "grip3", "grip4"])
        # self.grip_config = self.subrobot.setConfig([0, 1, 0, -1, 0])
        self.current_trimesh = None

    def init_local_pos(self):
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

    def ikSolver(self, robot, cspace, obj_pt, obj_index, rot):
        """Returns an IK solver that places the hand center at obj_pt with
        the palm oriented along obj_axis"""
        numAngles = 6
        step = 0.01
        solution = self.goal_transform_transit(robot, cspace, obj_pt, obj_index, numAngles, step)

        if solution is not None and solution is not False:
            grasp, grasparm = solution
            return grasp, grasparm
        else:
            return solution

    def goal_transform_transit(self, robot, cspace, obj_pt, obj_index, num, step):
        """
        Rotates and translates the gripper and tries grasping from each side specified by 3 angles
        :param robot: Robot model used
        :param cspace: CSpace object with defined feasibility
        :param obj_pt: Coordinates of the object (point in the world)
        :param obj_index: index of the object
        :param num: number of angles
        :param step: Sets the step of each translation. I.e: step = 0.01 => each translation is 0.01 farther
        :return: returns the configurations grasp and grasparm,  or None
        """
        global goal_config_transit
        global goal_rotation_transfer
        if num == 0:
            print("n parameter must be positive integer")
            cspace.close()
            return None
        i_old, j_old, k_old = 0, 0, 0
        sens = int(self.worldPositionRadius // step)
        l_old, n_old, m_old = -sens, -sens, -sens

        if goal_config_transit is not None:
            i_old = goal_config_transit[0]
            j_old = goal_config_transit[1]
            k_old = goal_config_transit[2]
            l_old = goal_config_transit[3]
            n_old = goal_config_transit[4]
            m_old = goal_config_transit[5]
        world_position_matrix = self.world_pos
        world_position_matrix.append(self.inside_mesh)
        # iterate through every angle and try to solve IKsolver task
        for i in range(i_old, 2 * num):
            angleX = ((i/num * math.pi), 0, 0)
            for j in range(j_old, 2 * num):
                angleY = (0, (j/num * math.pi), 0)
                for k in range(k_old, 2 * num):
                    angleZ = (0, 0, (k/num * math.pi))
                    self.angles = [angleX[0], angleY[1], angleZ[2]]

                    # creates rotation matrix for specified angle
                    rot_matrixX = euler_angle_to_rotation(angleX)
                    rot_matrixY = euler_angle_to_rotation(angleY)
                    rot_matrixZ = euler_angle_to_rotation(angleZ)

                    # rotates the object in three directions
                    rotObjX = [so3.apply(rot_matrixX, vectorops.sub(world_position_matrix[x],
                                                                    self.object.getTransform()[1]))
                               for x
                               in range(5)]
                    rotObjY = [so3.apply(rot_matrixY, rotObjX[h]) for h in range(5)]
                    rotObj_final = [so3.apply(rot_matrixZ, rotObjY[u]) for u in range(5)]

                    # store the position of the object in the world's coordinate system
                    for z in range(5):
                        world_position_matrix[z] = vectorops.add(obj_pt, rotObj_final[z])

                    for l in range(l_old, sens):
                        for n in range(n_old, sens):
                            for m in range(m_old, sens):
                                goal_rotation_transfer = None
                                world_pos_tmp = self.goal_translation(world_position_matrix, [l, n, m], step)
                                self.world_pos = world_pos_tmp[:4]
                                self.inside_mesh = world_pos_tmp[4]
                                if self.current_trimesh.contains([self.inside_mesh]):
                                    # set the goal to match local points on the arm with the object points
                                    goal = ik.objective(robot.link("tool0"),
                                                        local=[self.local_pos[0], self.local_pos[1], self.local_pos[2]],
                                                        world=[self.world_pos[1], self.world_pos[0], self.world_pos[2]])
                                    res = self.solve(robot, cspace, obj_index,  goal)
                                    if res is not None:
                                        m += 1
                                        if m >= sens:
                                            n += 1
                                            m = 0
                                            if n >= sens:
                                                l += 1
                                                n = 0
                                                if l >= sens:
                                                    k += 1
                                                    l = 0
                                                    if k >= 2*n:
                                                        j += 1
                                                        k = 0
                                                        if j >= 2*n:
                                                            i += 1
                                                            j = 0
                                        goal_config_transit = [i, j, k, l, n, m]
                                        return res
                                else:
                                    print("The point is outside the mesh")
        print("Solution to the transit problem hasn't been found")
        return False

    def goal_translation(self, world_positions, direction_indeces, step):
        final_points = []
        for point in world_positions:
            trans = vectorops.mul(direction_indeces, step)
            point = vectorops.add(point, trans)
            final_points.append(point)
        return final_points

    def solve(self, robot, cspace, obj_index, goal):
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


class Globals:
    def __init__(self, world):
        self.world = world
        self.robot = world.robot(0)
        self.collider = WorldCollider(world)


class TransitCSpace(CSpace):
    """A CSpace defining the feasibility constraints of the robot"""

    def __init__(self, globals, hand):
        CSpace.__init__(self)
        self.globals = globals
        self.robot = globals.robot
        self.hand = hand
        self.obj_index = hand.object.index
        # initial whole-body configuratoin
        self.q0 = self.robot.getConfig()

        # setup CSpace sampling range
        qlimits = list(zip(*self.robot.getJointLimits()))
        self.bound = [qlimits[i] for i in self.hand.armIndices]

        # setup CSpace edge checking epsilon
        self.eps = 1e-2

    def graspCheck(self):
        world = self.globals.world
        collider = self.globals.collider
        # test robot-object collisions
        for o in range(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index, o)):
                if o == object:
                    return True
        return False

    def feasible(self, x):
        # check arm joint limits
        for (xi, bi) in zip(x, self.bound):
            if xi < bi[0] or xi > bi[1]:
                return False

        # set up whole body configuration to test environment and self collisions
        q = self.q0[:]
        for i, xi in zip(self.hand.armIndices, x):
            q[i] = xi
        self.robot.setConfig(q)
        world = self.globals.world
        collider = self.globals.collider

        # test robot-object collisions
        for o in range(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index, o)):
                if o == self.obj_index:
                    world.rigidObject(o).appearance().setSilhouette(1, 0, 1, 0, 1)

                    cont_elem_obj_fing1 = world.rigidObject(o).geometry().contacts(world.robot(0).link(17).geometry(), 0, 0).elems1

                    for i in range(len(cont_elem_obj_fing1)):
                        print("Contact of the object with the 1st finger is in the index ", cont_elem_obj_fing1[i])
                        if 0 <= cont_elem_obj_fing1[i] < self.hand.object.geometry().numElements():
                            contact_color = self.hand.object.appearance().getElementColor(3, cont_elem_obj_fing1[i])
                            print("Color of the contact face", contact_color)
                            if np.array_equal([1.0, 0.0, 0.0, 1.0], contact_color):
                                return False

                    cont_elem_obj_fing2 = world.rigidObject(o).geometry().contacts(world.robot(0).link(19).geometry(), 0, 0).elems1
                    for i in range(len(cont_elem_obj_fing2)):
                        print("Contact of the object with the 2st finger is in the index ", cont_elem_obj_fing2[i])
                        if 0 <= cont_elem_obj_fing2[i] < self.hand.object.geometry().numElements():
                            contact_color = self.hand.object.appearance().getElementColor(3, cont_elem_obj_fing2[i])
                            print("Color of the contact face", contact_color)
                            if np.array_equal([1.0, 0.0, 0.0, 1.0], contact_color):
                                return False
                else:
                    return False

        # test robot-terrain collisions
        for o in range(world.numTerrains()):
            if any(collider.robotTerrainCollisions(self.robot.index, o)):
                return False

        # test robot self-collisions and paint collided links red
        if any(collider.robotSelfCollisions(self.robot.index)):
            collision_link1 = list(collider.robotSelfCollisions(self.robot.index))[0][0].getName()
            collision_link2 = list(collider.robotSelfCollisions(self.robot.index))[0][1].getName()
            vis.setColor(('world', 'kuka', collision_link1), 255, 0, 0, a=1.0)
            vis.setColor(('world', 'kuka', collision_link2), 255, 0, 0, a=1.0)
            return False

        return True


class GLPickAndPlacePlugin(GLPluginInterface):
    def __init__(self, world):

        GLPluginInterface.__init__(self)
        self.world = world
        self.trimeshes = self.upload_constraints()
        self.robot = world.robot(0)

        # start robot config
        self.qstart = self.robot.getConfig()

        # initialize the kuka arm
        self.hand = Hand(self.world, 'kuka')

        if len(self.trimeshes) > 0:
            self.object = world.rigidObject(0)
            #self.collis_object = world.rigidObject(1)

            # start object transform
            self.Tstart = self.object.getTransform()
            #self.Tstart_col = self.collis_object.getTransform()

            # initialize the trimesh, that's currently being evaluated
            self.hand.current_trimesh = self.trimeshes[self.object.index]
        else:
            self.object = None
            #self.collis_object = None
            self.Tstart = None
            self.Tstart_col = None

        # grasp transformation for object and collision area
        self.Tgrasp = None
        #self.Tgrasp_col = None

        # end configuration of the object and collision area
        self.Tgoal = None
        #self.Tgoal_col = None

        # initialize solution to planning problem
        self.transitPath = None
        self.transferPath = None
        self.retractPath = None
        self.objectPath = None
        #self.object_colPath = None
        self.path = None
        self.data = []
        self.force_denied = 0

    def display(self):
        # draw points on the robot
        kukaarm = self.hand
        glDisable(GL_LIGHTING)
        glPointSize(5.0)
        glDisable(GL_DEPTH_TEST)
        glBegin(GL_POINTS)

        # points connected by IKSolver
        glColor3f(0, 1, 0)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[0]))
        glColor3f(0, 0, 1)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[1]))
        glColor3f(1, 0, 0)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[2]))
        glColor3f(0, 0, 1)
        glVertex3fv(kukaarm.world_pos[0])
        glColor3f(0, 1, 0)
        glVertex3fv(kukaarm.world_pos[1])
        glColor3f(1, 0, 0)
        glVertex3fv(kukaarm.world_pos[2])
        glColor3f(0.5, 0.5, 0.5)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.GripperPoint1))
        glColor3f(0.5, 0.5, 0.5)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.GripperPoint2))

        # points on the terrain
        glColor3f(0.5, 0.3, 0)
        glVertex3fv([0.5, 0.2, 0.265])
        glColor3f(0.5, 0.3, 1)
        glVertex3fv([0.5, -0.05, 0.265])
        glColor3f(0.5, 1, 0.5)
        glVertex3fv([-0.6, -0.05, 0.265])
        glColor3f(0.5, 0.3, 0)
        glVertex3fv([-0.6, 0.2, 0.265])
        glEnd()
        glEnable(GL_DEPTH_TEST)

    def keyboardfunc(self, key, x, y):
        global goal_config_transit
        global goal_rotation_transfer
        h = 0.27
        if key == 'a':
            self.planTask(self.object.getTransform()[1], start_rot=False)
            return True

        if key == 'n':
            print("Moving to next action")
            if self.transitPath and self.transferPath and self.retractPath:
                # update start state
                self.qstart = self.retractPath[-1]
                self.Tstart = self.Tgoal
                self.Tstart_col = self.Tgoal_col
                self.robot.setConfig(self.qstart)
                self.object.setTransform(*self.Tstart)
                self.collis_object.setTransform(*self.Tstart_col)
                self.transitPath = None
                self.transferPath = None
                self.Tgrasp = None
                self.hand.refresh_world_coord()
                self.refresh()

        if key == '1':
            self.change_object(0)
        if key == '2':
            self.change_object(2)
        if key == '3':
            self.change_object(4)
        if key == '4':
            self.change_object(6)

        if key == 's':
            self.save_to_csv()

        if key == 'l':
            self.data_lenght()

    def planTask(self, dest, start_rot):
        global goal_config_transit
        global goal_rotation_transfer
        goal_config_transit = None
        goal_rotation_transfer = None
        while True:
            # set object's starting position
            self.object.setTransform(*self.Tstart)

            # set robot's starting configuration
            self.transitPath = None
            self.robot.setConfig(self.qstart)
            self.hand.refresh_world_coord()

            # plan transit path to grasp object
            self.transitPath = planTransit(self.world, self.object.index, self.hand)
            if self.transitPath is not None:
                if self.transitPath is False:
                    break
                self.transit_uti()
                if self.force_closure():
                    for pos in self.hand.world_pos:
                        data_arr = vectorops.sub(pos, dest)
                        self.data.append(data_arr)
                    self.data.append(self.hand.angles)
                else:
                   continue
            else:
                continue
        return True

    def transit_uti(self):
        self.Tgrasp = graspTransform(self.robot, self.hand, self.transitPath[-1], self.Tstart)
        #self.Tgrasp_col = graspTransform(self.robot, self.hand, self.transitPath[-1], self.Tstart_col)
        self.robot.setConfig(self.transitPath[-1])
        return 0

    def transfer_uti(self):
        self.Tgoal = graspedObjectTransform(self.robot, self.hand, self.transferPath[0], self.Tstart,
                                            self.transferPath[-1])
        # self.Tgoal_col = graspedObjectTransform(self.robot, self.hand, self.transferPath[0],
        #                                         self.Tstart_col,
        #                                         self.transferPath[-1])
        self.robot.setConfig(self.transferPath[-1])
        self.object.setTransform(*self.Tgoal)
        #self.collis_object.setTransform(*self.Tgoal_col)
        return 0

    def build_object_trajectory(self, resolution=0.05):
        xfertraj = RobotTrajectory(self.robot, range(len(self.transferPath)), self.transferPath)
        xferobj = xfertraj.getLinkTrajectory(self.hand.link, resolution)
        xferobj.postTransform(self.Tgrasp)
        #xferobj_col = xfertraj.getLinkTrajectory(self.hand.link, resolution)
        #xferobj_col.postTransform(self.Tgrasp_col)
        # offset times to be just during the transfer stage
        for i in range(len(xferobj.times)):
            xferobj.times[i] += len(self.transitPath)
        #for i in range(len(xferobj_col.times)):
        #    xferobj_col.times[i] += len(self.transitPath)
        self.objectPath = xferobj
        #self.object_colPath = xferobj_col
        return 0

    def task_animation(self):
        vis.animate(("world", self.robot.getName()), self.path, endBehavior='halt')
        vis.animate(("world", self.object.getName()), self.objectPath, endBehavior='halt')
        #vis.animate(("world", self.collis_object.getName()), self.object_colPath, endBehavior="halt")
        return 0

    def animate_none(self):
        vis.animate(("world", self.robot.getName()), None, endBehavior='halt')
        vis.animate(("world", self.object.getName()), None, endBehavior='halt')
        #vis.animate(("world", self.collis_object.getName()), None, endBehavior='halt')

    def change_object(self, obj_index):
        self.object = world.rigidObject(obj_index)
        self.Tstart = self.object.getTransform()
        self.hand.change_the_goal(obj_index)
        self.hand.refresh_world_coord()
        return True

    def force_closure(self):
        geometry_contacts = self.object.geometry().contacts(self.robot.link(19).geometry(), 0.01, 0.01)
        points1 = asarray(geometry_contacts.points1)
        normal = asarray(geometry_contacts.normals)
        contact_array = []
        for i in range(len(points1)):
            tmp_contact = contact.ContactPoint(points1[i], normal[i], self.object.getContactParameters().kFriction+3)
            force_closure = contact.forceClosure(tmp_contact)
            contact_array.append(force_closure)
        if any(contact_array):
            return True
        else:
            self.force_denied += 1
            return False

    def save_to_csv(self):
        cube_grasp_utility = self.data
        print(cube_grasp_utility)
        print(asarray(cube_grasp_utility))
        savetxt(save_directory, asarray(cube_grasp_utility), delimiter=',')
        return True

    def data_lenght(self):
        print("Dataset consists of ", len(self.data)/5, "elements")
        return

    def upload_constraints(self):
        trimeshes = []
        scale = [1, 1, 1]
        print("Please, enter the number of constraints to build")
        constraints_num = int(input())
        print("Building "+str(constraints_num)+" objects")
        for i in range(constraints_num):
            print("Object number "+str(i))
            print("Choose the object to upload:")
            print("1. Donut with more than one hole")
            print("2. Cursed Cube")
            print("3. Flashlight")
            print("4. Corona Beer")
            print("5. Poopie")
            object_num = int(input())
            if object_num == 1:
                way = "../data/objects/donut.obj"
                scale = [1, 1, 1]
            elif object_num == 2:
                way = "../data/objects/cursed_cube.obj"
                scale = [0.04, 0.04, 0.04]
            elif object_num == 3:
                way = "../data/objects/Flashlight.obj"
                scale = [0.03, 0.03, 0.03]
            elif object_num == 4:
                way = "../data/objects/corona.obj"
                scale = [0.003, 0.003, 0.003]
            elif object_num == 5:
                way = "../data/objects/poopie.obj"
                scale = [0.03, 0.03, 0.03]
            print(" Choose the position of the object (type point in world coordinates ex. [1, 1, 1] or any letter to use prepared ones)")
            position_inp = input()
            if type(position_inp) is list:
                dest = position_inp
            else:
                if i == 0:
                    dest = [0.5, 0.2, 0.27]
                elif i == 1:
                    dest = [0.5, 0, 0.27]
                elif i == 2:
                    dest = [0.2, 0.4, 0.07]
                elif i == 3:
                    dest = [-0.4, -0.4, 0.07]
                else:
                    dest = [0.5, i+0.15, 0.27]

            own_mesh = trimesh.exchange.load.load(way)
            filled = trimesh.repair.fill_holes(own_mesh)
            print(filled)
            print(own_mesh.is_watertight)

            m = TriangleMesh()
            for j in np.ndarray.flatten(own_mesh.vertices):
                m.vertices.append(j)

            for j in np.ndarray.flatten(own_mesh.faces):
                m.indices.append(int(j))



            new_mesh = world.loadRigidObject("../data/objects/block.obj")
            #new_coll = world.loadRigidObject("../data/objects/block.obj")
            new_mesh.setName("my_mesh"+str(i))
            #new_coll.setName("coll"+str(i))
            new_mesh.geometry().setTriangleMesh(m)
            new_mesh.setTransform([0, 1, 0, 0, 0, 1, 1, 0, 0], dest)
            #new_coll.setTransform([1, 0, 0, 0, 1, 0, 0, 0, 1], vectorops.add(dest, [-0.02, 0.044, 0]))
            new_mesh.geometry().scale(scale[0], scale[1], scale[2])
            #new_coll.geometry().scale(0.09, 0.002, 0.18)

            #new_coll.appearance().setColor(1, 0, 0, 1)
            #new_mesh.appearance().setColor(1, 0, 0, 1)
            color_arr = fill_colors(new_mesh, 6)
            trimeshes.append(own_mesh)
        vis.lock()
        vis.remove("world")
        vis.add("world", self.world)
        vis.colorize.colorize(new_mesh, color_arr)
        #vis.colorize.colorize(new_mesh, 'index', 'random', 'faces')
        vis.unlock()
        return trimeshes


class TransferCSpace(CSpace):
    def __init__(self, globals, hand, object):
        CSpace.__init__(self)
        self.globals = globals
        self.robot = globals.robot
        self.hand = hand
        self.object = object

        # initial whole-body configuration
        self.q0 = self.robot.getConfig()

        # setup initial grasp transform
        Tobj0 = object.getTransform()
        self.Tgrasp = graspTransform(self.robot, hand, self.q0, Tobj0)

        # setup CSpace sampling range
        qlimits = list(zip(*self.robot.getJointLimits()))
        self.bound = [qlimits[i] for i in self.hand.armIndices]

        # setup CSpace edge checking epsilon
        self.eps = 1e-2

    def objectTransform(self, x):
        """Given an arm configuration x, returns the object transformation"""
        q = self.q0[:]
        for i, xi in zip(self.hand.armIndices, x):
            q[i] = xi
        self.robot.setConfig(q)
        Thand = self.robot.link(self.hand.link).getTransform()
        return se3.mul(Thand, self.Tgrasp)

    def feasible(self, x):
        # check arm joint limits
        for (xi, bi) in zip(x, self.bound):
            if xi < bi[0] or xi > bi[1]:
                return False

        # set up whole body configuration to test environment and self collisions
        q = self.q0[:]
        for i, xi in zip(self.hand.armIndices, x):
            q[i] = xi
        self.robot.setConfig(q)
        world = self.globals.world
        collider = self.globals.collider
        grasped_object = self.hand.object.index

        # test robot-object collisions
        for o in range(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index, o)):
                if o == grasped_object:
                    world.rigidObject(o).appearance().setSilhouette(1, 0, 1, 0, 1)
                else:
                    return False

        # test object-object collisions
        # for o in range(world.numRigidObjects()):
        #     if any(collider.objectObjectCollisions(o, None)):
        #         return False

        # if collider.objectObjectCollisions(0, 2):
        #     print("TUT")
        #     return False

        # test object-terrain collisions
        for o in range(world.numRigidObjects()):
            if any(collider.objectTerrainCollisions(o, None)):
                return False

        # test robot-terrain collisions
        for o in range(world.numTerrains()):
            if any(collider.robotTerrainCollisions(self.robot.index, o)):
                return False

        # test robot self-collisions
        if any(collider.robotSelfCollisions(self.robot.index)):
            collision_link1 = list(collider.robotSelfCollisions(self.robot.index))[0][0].getName()
            collision_link2 = list(collider.robotSelfCollisions(self.robot.index))[0][1].getName()
            vis.setColor(('world', 'kuka', collision_link1), 255, 0, 0, a=1.0)
            vis.setColor(('world', 'kuka', collision_link2), 255, 0, 0, a=1.0)
            return False
        return True


def graspTransform(robot, hand, qrobot0, Tobj0):
    """Given initial robot configuration qrobot0 and object transform Tobj0,
    returns the grasp transformation Tgrasp, which produces the object transform
    via the composition  Tobj = Thand * Tgrasp"""
    robot.setConfig(qrobot0)
    Thand0 = robot.link(hand.link).getTransform()
    Tgrasp = se3.mul(se3.inv(Thand0), Tobj0)
    return Tgrasp


def graspedObjectTransform(robot, hand, qrobot0, Tobj0, qrobot):
    """Given initial robot configuration qrobot0 and object transform Tobj0,
    returns the object transformation corresponding to new configuration
    qrobot assuming the object is rigidly attached to the hand"""
    Tgrasp = graspTransform(robot, hand, qrobot0, Tobj0)
    robot.setConfig(qrobot)
    Thand = robot.link(hand.link).getTransform()
    return se3.mul(Thand, Tgrasp)


def planTransit(world, objectIndex, hand):
    globals = Globals(world)
    cspace = TransitCSpace(globals, hand)
    obj = world.rigidObject(objectIndex)
    robot = world.robot(0)
    # get the start config
    q0 = robot.getConfig()
    q0arm = [q0[i] for i in hand.armIndices]

    if not cspace.feasible(q0arm):
        print("Warning, arm start configuration is infeasible")
        #cspace.close()
        return None

    print("Trying to find pregrasp config...")
    solution = hand.ikSolver(robot, cspace, obj.getTransform()[1], obj.index, False)

    if solution is not None and solution is not False:
        qpregrasp, qpregrasparm = solution
    else:
        return solution

    print("Planning transit motion to pregrasp config...")
    MotionPlan.setOptions(connectionThreshold=5.0, perturbationRadius=0.5)
    algorithm = "sbl"
    planner = MotionPlan(cspace, algorithm)
    planner.setEndpoints(q0arm, qpregrasparm)

    iters = 0
    step = 10
    while planner.getPath() is None and iters < 1000:
        planner.planMore(step)
        iters += step

    if planner.getPath() is None:
        print("Failed finding transit path...")
        return None
    print("Success, found path with", len(planner.getPath()), "milestones...")
    # lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi, i in zip(qarm, hand.armIndices):
            path[-1][i] = qi

    for i in range(1, 101):
        qarm_tmp = path + [hand.open(path[-1], 1 - (i / 100))]
        final_qarm = qarm_tmp[-1]
        robot.setConfig(final_qarm)
        if any(globals.collider.robotObjectCollisions(robot.index, obj.index)):
            break

    if not cspace.feasible(final_qarm):
        print("Fragile area is involved")
        return None

    # add a path to the grasp configuration
    return qarm_tmp


def euler_angle_to_rotation(ea, convention='zyx'):
    """Converts an euler angle representation to a rotation matrix.
    Can use arbitrary axes specified by the convention
    arguments (default is 'zyx', or roll-pitch-yaw euler angles).  Any
    3-letter combination of 'x', 'y', and 'z' are accepted.
    """
    axis_names_to_vectors = dict([('x', (1, 0, 0)), ('y', (0, 1, 0)), ('z', (0, 0, 1))])
    axis0, axis1, axis2 = convention
    R0 = so3.rotation(axis_names_to_vectors[axis0], ea[0])
    R1 = so3.rotation(axis_names_to_vectors[axis1], ea[1])
    R2 = so3.rotation(axis_names_to_vectors[axis2], ea[2])
    return so3.mul(R0, so3.mul(R1, R2))


if __name__ == "__main__":
    world = WorldModel()
    collider = WorldCollider(world)

    res = world.readFile("../Python/exercises/manipulation/grasp_attempt.xml")
    if not res: raise RuntimeError("Unable to load world file")
    vis.add("world", world)
    vis.setWindowTitle("Pick and place test, use a/b/c/d to select target")
    vis.pushPlugin(GLPickAndPlacePlugin(world))
    vis.setColor(('world', 'kuka', world.robot(0).link(17).getName()), 0, 255, 0, a=1.0)
    vis.setColor(('world', 'kuka', world.robot(0).link(19).getName()), 0, 255, 0, a=1.0)
    vis.show()

    while vis.shown():
        time.sleep(0.1)
    vis.setPlugin(None)
    vis.kill()
