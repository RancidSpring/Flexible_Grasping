from klampt import *
from klampt.model import contact
from klampt.model.create import *
from klampt.model import ik
from klampt.model.collide import WorldCollider
from klampt.model.trajectory import RobotTrajectory
from klampt.vis.glcommon import *
from klampt import vis
from klampt.math import vectorops, so3, se3
from klampt.plan.cspace import CSpace, MotionPlan
from openkukagrip import openhand
from OpenGL.GL import *

from numpy import asarray
from numpy import savetxt
import math
import time

# global variable used to store the latest tried config of the arm in case path wasn't found
goal_config_transit = None
goal_rotation_transfer = None


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
        self.localPosition1 = (0.01, 0.01, 0.15)
        self.localPosition2 = (-0.01, 0.01, 0.15)
        self.localPosition3 = (0, -0.01, 0.15)
        self.worldPositionRadius = 0.03
        self.GripperPoint1 = (0.0, 0.043, 0.1275)
        self.GripperPoint2 = (0.0, -0.043, 0.1275)
        self.worldPosition1 = vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0])
        self.worldPosition2 = vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0])
        self.worldPosition3 = vectorops.add(self.object.getTransform()[1], [0, -0.01, 0])
        self.subrobot = model.subrobot.SubRobotModel(self.world.robot(0), ["grip0", "grip1", "grip2", "grip3", "grip4"])
        self.grip_config = self.subrobot.setConfig([0, 1, 0, -1, 0])

    def open(self, q, amount=1.0):
        """Computes a configuration identical to q but with hand open
        if amount = 1 or closed if amount = 0"""
        return openhand(q, self.hand, amount)

    def ikSolver(self, robot, cspace, obj_pt, transfer, rot):
        """Returns an IK solver that places the hand center at obj_pt with
        the palm oriented along obj_axis"""
        numAngles = 6
        step = 0.01
        if not transfer:
            solution = self.goal_transform_transit(robot, cspace, obj_pt, numAngles, step)
        else:
            solution = self.goal_rotation_transfer(robot, cspace, obj_pt, rot)

        if solution is not None:
            grasp, grasparm = solution
            return grasp, grasparm
        else:
            return None

    def goal_transform_transit(self, robot, cspace, obj_pt, n, step):
        """
        Rotates and translates the gripper and tries grasping from each side specified by 3 angles
        :param robot: Robot model used
        :param cspace: CSpace object with defined feasibility
        :param obj_pt: Coordinates of the object (point in the world)
        :param n: Used to set the step of each rotation. I.e: n = 3 => each step is pi/3
        :param step: Used to set the step of each translation. I.e: step = 0.01 => each translation is 0.01 farther
        :return: returns the configurations grasp and grasparm,  or None
        """
        global goal_config_transit
        global goal_rotation_transfer

        if n == 0:
            print("n parameter must be positive integer")
            cspace.close()
            return None
        i_old = 0
        j_old = 0
        k_old = 0
        sens = int(self.worldPositionRadius // step)
        l_old = -sens
        m_old = -sens

        if goal_config_transit is not None:
            i_old = goal_config_transit[0]
            j_old = goal_config_transit[1]
            k_old = goal_config_transit[2]
            l_old = goal_config_transit[3]
            m_old = goal_config_transit[5]
        world_position_matrix = [self.worldPosition1, self.worldPosition2, self.worldPosition3]
        # iterate through every angle and try to solve IKsolver task
        for i in range(i_old, 2 * n):
            angleX = (i * math.pi / n, 0, 0)
            for j in range(j_old, 2 * n):
                angleY = (0, j * math.pi / n, 0)
                for k in range(k_old, 2 * n):
                    angleZ = (0, 0, k * math.pi / n)
                    # creates rotation matrix for specified angle
                    rot_matrixX = euler_angle_to_rotation(angleX)
                    rot_matrixY = euler_angle_to_rotation(angleY)
                    rot_matrixZ = euler_angle_to_rotation(angleZ)
                    # rotates the object in three directions
                    rotObjX = [so3.apply(rot_matrixX, vectorops.sub(world_position_matrix[x],
                                                                    self.object.getTransform()[1]))
                               for x
                               in range(3)]
                    rotObjY = [so3.apply(rot_matrixY, rotObjX[h]) for h in range(3)]
                    rotObj_final = [so3.apply(rot_matrixZ, rotObjY[u]) for u in range(3)]

                    # store the position of the object in the world's coordinate system
                    self.worldPosition1 = vectorops.add(obj_pt, rotObj_final[0])
                    self.worldPosition2 = vectorops.add(obj_pt, rotObj_final[1])
                    self.worldPosition3 = vectorops.add(obj_pt, rotObj_final[2])

                    world_positions = [self.worldPosition1, self.worldPosition2, self.worldPosition3]
                    for l in range(l_old, sens):
                        for m in range(m_old, sens):
                            goal_rotation_transfer = None
                            final_position = self.goal_translation(world_positions, [l, 0, m], step)
                            self.worldPosition1 = final_position[0]
                            self.worldPosition2 = final_position[1]
                            self.worldPosition3 = final_position[2]
                            # set the goal to match local points on the arm with the object points
                            goal = ik.objective(robot.link("tool0"),
                                                local=[self.localPosition1, self.localPosition2, self.localPosition3],
                                                world=[self.worldPosition2, self.worldPosition1, self.worldPosition3])
                            res = self.solve(robot, cspace, goal)
                            if res is not None:
                                m += 1
                                if m >= sens:
                                    l += 1
                                    m = 0
                                    if l >= sens:
                                        k += 1
                                        l = 0
                                    if k >= 2*n:
                                        j += 1
                                        k = 0
                                        if j >= 2*n:
                                            i += 1
                                            j = 0
                                goal_config_transit = [i, j, k, l, 0, m]
                                return res
        print("Solution to the transit problem hasn't been found")
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

        i_old = 0
        j_old = 0
        k_old = 0

        if goal_rotation_transfer is not None:
            i_old = goal_rotation_transfer[0]
            j_old = goal_rotation_transfer[1]
            k_old = goal_rotation_transfer[2]

        world_position_matrix = [self.worldPosition1, self.worldPosition2, self.worldPosition3]
        # iterate through every angle and try to solve IKsolver task
        for i in range(i_old, 4):
            angleX = (i * math.pi / 2, 0, 0)
            for j in range(j_old, 4):
                angleY = (0, j * math.pi / 2, 0)
                for k in range(k_old, 4):
                    angleZ = (0, 0, k * math.pi / 2)
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
                                        local=[self.localPosition1, self.localPosition2, self.localPosition3],
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

    def goal_translation(self, world_positions, direction_indeces, step):
        final_points = []
        for point in world_positions:
            trans = vectorops.mul(direction_indeces, step)
            point = vectorops.add(point, trans)
            final_points.append(point)
        return final_points

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
                    return grasp, grasparm
            if grasp is None:
                # print("Grasp solve failed")
                pass

    def refresh_world_coord(self):
        self.worldPosition1 = vectorops.add(self.object.getTransform()[1], [0.01, 0.01, 0])
        self.worldPosition2 = vectorops.add(self.object.getTransform()[1], [-0.01, 0.01, 0])
        self.worldPosition3 = vectorops.add(self.object.getTransform()[1], [0, -0.01, 0])
        return 0

    def change_the_goal(self, obj_index):
        self.object = world.rigidObject(obj_index)
        return 0


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

        # initial whole-body configuratoin
        self.q0 = self.robot.getConfig()

        # setup CSpace sampling range
        qlimits = list(zip(*self.robot.getJointLimits()))
        self.bound = [qlimits[i] for i in self.hand.armIndices]

        # setup CSpace edge checking epsilon
        self.eps = 1e-2

    def graspCheck(self, object):
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
        grasped_object = self.hand.object.index

        # test robot-object collisions
        for o in range(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index, o)):
                if o == grasped_object:
                    world.rigidObject(o).appearance().setSilhouette(1, 0, 1, 0, 1)
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
        self.robot = world.robot(0)
        self.object = world.rigidObject(0)
        self.collis_object = world.rigidObject(1)

        # start robot config
        self.qstart = self.robot.getConfig()

        # start object transform
        self.Tstart = self.object.getTransform()
        self.Tstart_col = self.collis_object.getTransform()

        # grasp transformation for object and collision area
        self.Tgrasp = None
        self.Tgrasp_col = None

        # end configuration of the object and collision area
        self.Tgoal = None
        self.Tgoal_col = None

        # initialize solution to planning problem
        self.transitPath = None
        self.transferPath = None
        self.retractPath = None
        self.objectPath = None
        self.object_colPath = None
        self.path = None

        # initialize the kuka arm
        self.hand = Hand(self.world, 'kuka')

        # geometry collision points
        self.geom_coll1 = [0.5, 0, 0.5]
        self.geom_coll2 = [0.5, 0, 0.5]

    def display(self):
        # draw points on the robot
        kukaarm = self.hand
        glDisable(GL_LIGHTING)
        glPointSize(5.0)
        glDisable(GL_DEPTH_TEST)
        glBegin(GL_POINTS)

        # points connected by IKSolver
        glColor3f(0, 1, 0)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.localPosition1))
        glColor3f(0, 0, 1)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.localPosition2))
        glColor3f(1, 0, 0)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.localPosition3))
        glColor3f(0, 0, 1)
        glVertex3fv(kukaarm.worldPosition1)
        glColor3f(0, 1, 0)
        glVertex3fv(kukaarm.worldPosition2)
        glColor3f(1, 0, 0)
        glVertex3fv(kukaarm.worldPosition3)
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

        # geom coll
        glColor3f(1, 1, 1)
        glVertex3fv(self.geom_coll1)
        glColor3f(1, 1, 1)
        glVertex3fv(self.geom_coll2)

        glEnd()
        glEnable(GL_DEPTH_TEST)

    def keyboardfunc(self, key, x, y):
        h = 0.27
        targets = {'a': [0.5, 0.2, h], 'b': [0.5, -0.05, h], 'c': [-0.6, -0.05, h], 'd': [-0.6, 0.2, h], 'e' : [-0.7, 0.2, h], 'f' : [-0.7, 0.1, h], 'g' : [-0.6, 0.1, h]}
        if key in targets:
            dest = targets[key]
            return self.planTask(dest, start_rot=False)

        if key == 'n':
            print("Moving to next action")
            if self.transitPath and self.transferPath and self.retractPath:
                # update start state
                self.qstart = self.retractPath[-1]
                self.Tstart = self.Tgoal
                self.Tstart_col = self.Tgoal_col
                self.robot.setConfig(self.qstart)
                print(self.Tstart)
                self.object.setTransform(*self.Tstart)
                self.collis_object.setTransform(*self.Tstart_col)
                self.transitPath = None
                self.transferPath = None
                self.Tgrasp = None
                self.hand.refresh_world_coord()
                self.refresh()

                vis.animate(("world", self.robot.getName()), None, endBehavior='halt')
                vis.animate(("world", self.object.getName()), None, endBehavior='halt')
                vis.animate(("world", self.collis_object.getName()), None, endBehavior='halt')

        if key == 's':
            vis.animate(("world", self.robot.getName()), self.path, endBehavior='halt')
            vis.animate(("world", self.object.getName()), self.objectPath, endBehavior='halt')
            vis.animate(("world", self.collis_object.getName()), self.object_colPath, endBehavior='halt')
        if key == 'l':
            return self.planTask(self.object.getTransform()[1], start_rot=True)
        if key == '1':
            self.change_object(0)
        if key == '2':
            self.change_object(2)
        if key == '3':
            self.change_object(4)
        if key == '4':
            self.change_object(6)

    def planTask(self, dest, start_rot):
        global goal_config_transit
        global goal_rotation_transfer
        goal_config_transit = None
        goal_rotation_transfer = None
        shift = vectorops.sub(dest, self.Tstart[1])
        iteration = 0
        while True:
            # set object's starting position
            self.object.setTransform(*self.Tstart)
            self.collis_object.setTransform(*self.Tstart_col)

            # set robot's starting configuration
            self.transitPath = None
            self.transferPath = None
            self.retractPath = None
            self.robot.setConfig(self.qstart)
            self.hand.refresh_world_coord()
            # plan transit path to grasp object
            self.transitPath = planTransit(self.world, self.object.index, self.hand)
            if self.transitPath:
                self.transit_uti()
                if not self.force_closure():
                    continue

                # plan transfer path with rigidly grasped object
                self.transferPath = planTransfer(self.world, self.object.index, self.hand, shift, start_rot)
                if self.transferPath:
                    self.transfer_uti()

                    # plan free path to retract arm
                    self.retractPath = planFree(self.world, self.hand, self.qstart)

            if self.transitPath and self.transferPath and self.retractPath:
                # build trajectory for the arm
                milestones = self.transitPath + self.transferPath + self.retractPath
                self.path = RobotTrajectory(self.robot, range(len(milestones)), milestones)

                # build trajectory for the object and collision area
                self.build_object_trajectory(0.05)

                # animate the given task
                self.task_animation()
                return True

            # end condition
            iteration += 1
            if iteration > 1000:
                break

        self.animate_none()
        return False

    def transit_uti(self):
        self.Tgrasp = graspTransform(self.robot, self.hand, self.transitPath[-1], self.Tstart)
        self.Tgrasp_col = graspTransform(self.robot, self.hand, self.transitPath[-1], self.Tstart_col)
        self.robot.setConfig(self.transitPath[-1])
        return 0

    def transfer_uti(self):
        self.Tgoal = graspedObjectTransform(self.robot, self.hand, self.transferPath[0], self.Tstart,
                                            self.transferPath[-1])
        self.Tgoal_col = graspedObjectTransform(self.robot, self.hand, self.transferPath[0],
                                                self.Tstart_col,
                                                self.transferPath[-1])
        self.robot.setConfig(self.transferPath[-1])
        self.object.setTransform(*self.Tgoal)
        self.collis_object.setTransform(*self.Tgoal_col)
        return 0

    def build_object_trajectory(self, resolution=0.05):
        xfertraj = RobotTrajectory(self.robot, range(len(self.transferPath)), self.transferPath)
        xferobj = xfertraj.getLinkTrajectory(self.hand.link, resolution)
        xferobj.postTransform(self.Tgrasp)
        xferobj_col = xfertraj.getLinkTrajectory(self.hand.link, resolution)
        xferobj_col.postTransform(self.Tgrasp_col)
        # offset times to be just during the transfer stage
        for i in range(len(xferobj.times)):
            xferobj.times[i] += len(self.transitPath)
        for i in range(len(xferobj_col.times)):
            xferobj_col.times[i] += len(self.transitPath)
        self.objectPath = xferobj
        self.object_colPath = xferobj_col
        return 0

    def task_animation(self):
        vis.animate(("world", self.robot.getName()), self.path, endBehavior='halt')
        vis.animate(("world", self.object.getName()), self.objectPath, endBehavior='halt')
        vis.animate(("world", self.collis_object.getName()), self.object_colPath, endBehavior="halt")
        return 0

    def animate_none(self):
        vis.animate(("world", self.robot.getName()), None, endBehavior='halt')
        vis.animate(("world", self.object.getName()), None, endBehavior='halt')
        vis.animate(("world", self.collis_object.getName()), None, endBehavior='halt')

    def force_closure(self):
        geometry_contacts = self.object.geometry().contacts(self.robot.link(19).geometry(), 0.01, 0.01)
        points1 = asarray(geometry_contacts.points1)
        normal = asarray(geometry_contacts.normals)
        contact_array = []
        for i in range(len(points1)):
            tmp_contact = contact.ContactPoint(points1[i], normal[i], self.object.getContactParameters().kFriction)
            force_closure = contact.forceClosure(tmp_contact)
            contact_array.append(force_closure)
        if any(contact_array):
            print(contact_array)
            return True
        else:
            return False

    def change_object(self, obj_index):
        self.object = world.rigidObject(obj_index)
        self.collis_object = world.rigidObject(obj_index+1)
        self.Tstart = self.object.getTransform()
        self.Tstart_col = self.collis_object.getTransform()
        self.hand.change_the_goal(obj_index)
        self.hand.refresh_world_coord()
        return True


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
    solution = hand.ikSolver(robot, cspace, obj.getTransform()[1], False, False)

    if solution is not None:
        qpregrasp, qpregrasparm = solution
    else:
        return None

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

    # add a path to the grasp configuration
    return path + [hand.open(path[-1], 0)]


def planTransfer(world, objectIndex, hand, shift, rot):
    """Plan a transfer path for the robot given in world, which is currently
    holding the object indexed by objectIndex in the hand hand.

    The desired motion should translate the object by shift without rotating
    the object.
    """
    globals = Globals(world)
    obj = world.rigidObject(objectIndex)
    cspace = TransferCSpace(globals, hand, obj)
    robot = world.robot(0)
    qmin, qmax = robot.getJointLimits()

    # get the start config
    q0 = robot.getConfig()
    q0arm = [q0[i] for i in hand.armIndices]
    if not cspace.feasible(q0arm):
        print("Warning, arm start configuration is infeasible")
        return None
    start = time.time()
    solution = hand.ikSolver(robot, cspace,
                                          vectorops.add(vectorops.add(obj.getTransform()[1], [0, 0, 0.0]), shift), True, rot)
    end = time.time()
    print("transfer time", end - start)
    if solution is not None:
        qungrasp, qungrasparm = solution
    else:
        return None
    # plan the transfer path between q0arm and qungrasparm
    print("Planning transfer motion to ungrasp config...")
    algorithm = "sbl"
    MotionPlan.setOptions(connectionThreshold=5.0, perturbationRadius=0.5)
    planner = MotionPlan(cspace, algorithm)
    planner.setEndpoints(q0arm, qungrasparm)

    iters = 0
    step = 10
    while planner.getPath() is None and iters < 1000:
        planner.planMore(step)
        iters += step

    if planner.getPath() is None:
        print("Failed finding transit path")
        return None
    print("Success, found path with", len(planner.getPath()), "milestones")

    # lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi, i in zip(qarm, hand.armIndices):
            path[-1][i] = qi

    qpostungrasp = hand.open(qungrasp, 0)
    return path + [qpostungrasp]


def planFree(world, hand, qtarget):
    """Plans a free-space motion for the robot's arm from the current
    configuration to the destination qtarget"""
    globals = Globals(world)
    cspace = TransitCSpace(globals, hand)
    robot = world.robot(0)
    qmin, qmax = robot.getJointLimits()

    # get the start/goal config
    q0 = robot.getConfig()
    q0arm = [q0[i] for i in hand.armIndices]
    qtargetarm = [qtarget[i] for i in hand.armIndices]

    if not cspace.feasible(q0arm):
        print("Warning, arm start configuration is infeasible")
        return None
    if not cspace.feasible(qtargetarm):
        print("Warning, arm goal configuration is infeasible")
        return None

    print("Planning transit motion to target config...")
    algorithm = "sbl"
    MotionPlan.setOptions(connectionThreshold=5.0, perturbationRadius=0.5)
    planner = MotionPlan(cspace, algorithm)
    planner.setEndpoints(q0arm, qtargetarm)
    iters = 0
    step = 10
    while planner.getPath() is None and iters < 1000:
        planner.planMore(step)
        iters += step
    cspace.close()
    if planner.getPath() is None:
        print("Failed finding transit path")
        return None
    print("Success")
    # lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi, i in zip(qarm, hand.armIndices):
            path[-1][i] = qi
    return path + [hand.open(path[-1], 1)]


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


def convert(geom, type, label):
    global a, b, atypes, btype
    if label == 'A':
        vis.add(label, atypes[type])
        vis.setColor(label, 1, 0, 0, 0.5)
        a = atypes[type]


def setMode(m):
    global mode, drawExtra
    mode = m
    vis.lock()
    for s in drawExtra:
        vis.remove(s)
    drawExtra = set()
    vis.unlock()


if __name__ == "__main__":
    world = WorldModel()
    collider = WorldCollider(world)

    res = world.readFile("../Python/exercises/manipulation/grasp_attempt.xml")
    if not res: raise RuntimeError("Unable to load world file")
    vis.add("world", world)
    vis.setWindowTitle("Pick and place test, use a/b/c/d to select target")
    vis.pushPlugin(GLPickAndPlacePlugin(world))
    vis.show()

    object_position = world.rigidObject(0).getTransform()[1]
    a = box(0.07, 0.09, 0.07, center=(object_position[0], object_position[1], object_position[2]),
            type='GeometricPrimitive')
    b = box(0.07, 0.09, 0.07, center=(-0.7, 0.2, 0.27),
            type='GeometricPrimitive')
    c = box(0.07, 0.09, 0.07, center=(-0.7, 0.1, 0.27),
            type='GeometricPrimitive')
    d = box(0.07, 0.09, 0.07, center=(-0.6, 0.2, 0.27),
            type='GeometricPrimitive')
    g = box(0.07, 0.09, 0.07, center=(-0.6, 0.1, 0.27),
            type='GeometricPrimitive')

    geomtypes = ['GeometricPrimitive', 'TriangleMesh', 'PointCloud', 'VolumeGrid']
    tparams = {'PointCloud': 0.05, 'VolumeGrid': 0.04}
    atypes = dict((t, a.convert(t, tparams.get(t, 0))) for t in geomtypes)
    btypes = dict((t, b.convert(t, tparams.get(t, 0))) for t in geomtypes)
    ctypes = dict((t, c.convert(t, tparams.get(t, 0))) for t in geomtypes)
    dtypes = dict((t, d.convert(t, tparams.get(t, 0))) for t in geomtypes)
    gtypes = dict((t, g.convert(t, tparams.get(t, 0))) for t in geomtypes)

    vis.add("A", atypes['GeometricPrimitive'])
    vis.setColor("A", 1, 1, 1, 0.5)
    vis.add("E", btypes['GeometricPrimitive'])
    vis.setColor("E", 0, 1, 0, 0.5)
    vis.add("F", ctypes['GeometricPrimitive'])
    vis.setColor("F", 0, 1, 0, 0.5)
    vis.add("D", dtypes['GeometricPrimitive'])
    vis.setColor("D", 0, 1, 0, 0.5)
    vis.add("G", gtypes['GeometricPrimitive'])
    vis.setColor("G", 0, 1, 0, 0.5)

    vis.addAction(lambda: setMode('collision'), 'Collision mode', 'c')

    mode = 'collision'
    drawExtra = set()
    while vis.shown():
        if mode == 'collision':
            if world.robot(0).link(17).geometry().collides(a):
                vis.setColor('A', 1, 1, 1, 0.8)
            else:
                vis.setColor('A', 0, 1, 0, 0.5)

            if world.robot(0).link(17).geometry().collides(b):
                vis.setColor('E', 1, 1, 1, 0.8)

            if world.robot(0).link(17).geometry().collides(c):
                vis.setColor('F', 1, 1, 1, 0.8)

            if world.robot(0).link(17).geometry().collides(d):
                vis.setColor('D', 1, 1, 1, 0.8)

            if world.robot(0).link(17).geometry().collides(g):
                vis.setColor('G', 1, 1, 1, 0.8)

        time.sleep(0.1)
    vis.setPlugin(None)
    vis.kill()

