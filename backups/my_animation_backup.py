import klampt
from klampt.vis import GLRealtimeProgram
from klampt.model.subrobot import RobotModelLink
from klampt import robotsim
from klampt import *
from klampt.model import ik
from klampt.model.contact import Hold
from klampt.model.collide import WorldCollider
from klampt.model.trajectory import RobotTrajectory
from klampt.model.subrobot import SubRobotModel
from klampt.vis.glcommon import *
from klampt import vis
from klampt.math import vectorops, so3, se3
from klampt.plan.cspace import CSpace, MotionPlan
from openkukagrip import openhand
from OpenGL.GL import *
import math
import random
import time
from pip._vendor.distlib.compat import raw_input


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
        self.gripperIndex = 14
        self.link = 14
        self.world = world
        self.localPosition1 = (0.01, 0.01, 0.15)
        self.localPosition2 = (-0.01, 0.01, 0.15)
        self.localPosition3 = (0, -0.01, 0.15)
        self.GripperPoint1 = (0.0, 0.043, 0.1275)
        self.GripperPoint2 = (0.0, -0.043, 0.1275)
        self.worldPosition1 = vectorops.add(self.world.rigidObject(0).getTransform()[1], [0.01, 0.01, 0])
        self.worldPosition2 = vectorops.add(self.world.rigidObject(0).getTransform()[1], [-0.01, 0.01, 0])
        self.worldPosition3 = vectorops.add(self.world.rigidObject(0).getTransform()[1], [0, -0.01, 0])
        self.subrobot = model.subrobot.SubRobotModel(self.world.robot(0), ["grip0", "grip1", "grip2", "grip3", "grip4"])
        self.grip_config = self.subrobot.setConfig([0, 1, 0, -1, 0])

    def open(self, q, amount=1.0):
        """Computes a configuration identical to q but with hand open
        if amount = 1 or closed if amount = 0"""
        return openhand(q, self.hand, amount)

    def ikSolver(self, robot, cspace, obj_pt):
        """Returns an IK solver that places the hand center at obj_pt with
        the palm oriented along obj_axis"""
        numAngles = 6
        grasp, grasparm = self.goal_rotation(robot, cspace, obj_pt, numAngles)
        return grasp, grasparm

    def goal_rotation(self, robot, cspace, obj_pt, n):
        """
        Rotates the gripper and tries grasping from each sides specified by 3 angles
        :param robot: Robot model used
        :param cspace: CSpace object with defined feasibility
        :param obj_pt: Coordinates of the object (point in the world)
        :param n: Used to set the step of each rotation. I.e: n = 3 => each step is pi/3
        :return: returns the configurations grasp and grasparm,  or None
        """
        if n == 0:
            print("n parameter must be positive integer")
            cspace.close()
            return None

        world_position_matrix = [self.worldPosition1, self.worldPosition2, self.worldPosition3]

        # iterate through every angle and try to solve IKsolver task
        for i in range(2 * n):
            angleX = (i * math.pi / n, 0, 0)
            for j in range(2 * n):
                angleY = (0, j * math.pi / n, 0)
                for k in range(2 * n):
                    angleZ = (0, 0, k * math.pi / n)

                    # creates rotation matrix for specified angle
                    rot_matrixX = euler_angle_to_rotation(angleX)
                    rot_matrixY = euler_angle_to_rotation(angleY)
                    rot_matrixZ = euler_angle_to_rotation(angleZ)

                    # rotates the object in three directions
                    rotObjX = [so3.apply(rot_matrixX, vectorops.sub(world_position_matrix[x],
                                                                    self.world.rigidObject(0).getTransform()[1])) for x
                               in range(3)]
                    rotObjY = [so3.apply(rot_matrixY, rotObjX[h]) for h in range(3)]
                    rotObj_final = [so3.apply(rot_matrixZ, rotObjY[u]) for u in range(3)]

                    # store the position of the object in the world's coordinate system
                    self.worldPosition1 = vectorops.add(obj_pt, rotObj_final[0])
                    self.worldPosition2 = vectorops.add(obj_pt, rotObj_final[1])
                    self.worldPosition3 = vectorops.add(obj_pt, rotObj_final[2])

                    # set the goal to match local points on the arm with the object points
                    goal = ik.objective(robot.link("tool0"),
                                        local=[self.localPosition1, self.localPosition2, self.localPosition3],
                                        world=[self.worldPosition2, self.worldPosition1, self.worldPosition3])
                    res = self.solve(robot, cspace, goal)
                    if res is not None:
                        return res

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
                else:
                    print(grasp, grasparm)
                    return grasp, grasparm
            if grasp is None:
                print("Grasp solve failed")


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

        # test robot-object collisions
        for o in range(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index, o)):
                if o != 0:
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


class GLPickAndPlacePlugin(GLPluginInterface):
    def __init__(self, world):
        GLPluginInterface.__init__(self)
        self.world = world
        self.robot = world.robot(0)
        self.object = world.rigidObject(0)

        # start robot config
        self.qstart = self.robot.getConfig()

        # start object transform
        self.Tstart = self.object.getTransform()

        # solution to planning problem
        self.transitPath = None
        self.transferPath = None
        self.retractPath = None
        self.animationTime = None

        # Initialize the kuka arm
        self.hand = Hand(self.world, 'kuka')

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
        # point on the terrain
        glColor3f(0.5, 0.3, 0)
        glVertex3fv([0.5, 0.2, 0.265])
        glColor3f(0.5, 0.3, 1)
        glVertex3fv([0.5, -0.05, 0.265])
        glColor3f(0.5, 1, 0.5)
        glVertex3fv([-0.5, -0.05, 0.265])
        glColor3f(0.5, 0.3, 0)
        glVertex3fv([-0.5, 0.2, 0.265])
        glEnd()
        glEnable(GL_DEPTH_TEST)

    def keyboardfunc(self, key, x, y):
        h = 0.265
        targets = {'a': (0.5, 0.2, h), 'b': (0.5, -0.05, h), 'c': (-0.5, -0.05, h), 'd': (-0.5, 0.2, h)}
        if key in targets:
            dest = targets[key]
            shift = vectorops.sub(dest, vectorops.add(self.Tstart[1], [0, 0, 0.2]))
            self.object.setTransform(*self.Tstart)
            self.animationTime = 1
            # plan transit path to grasp object
            self.transitPath = planTransit(self.world, 0, self.hand)
            if self.transitPath:
                # plan transfer path
                self.Tgrasp = graspTransform(self.robot, self.hand, self.transitPath[-1], self.Tstart)
                self.robot.setConfig(self.transitPath[-1])
                self.transferPath = planTransfer(self.world, 0, self.hand, shift)
                if self.transferPath:
                    self.Tgoal = graspedObjectTransform(self.robot, self.hand, self.transferPath[0], self.Tstart,
                                                        self.transferPath[-1])
                    # plan free path to retract arm
                    self.robot.setConfig(self.transferPath[-1])
                    self.object.setTransform(*self.Tgoal)
                    self.retractPath = planFree(self.world, self.hand, self.qstart)
            # reset the animation
            if self.transitPath and self.transferPath and self.retractPath:
                milestones = self.transitPath + self.transferPath + self.retractPath
                self.path = RobotTrajectory(self.robot, range(len(milestones)), milestones)
                resolution = 0.05
                xfertraj = RobotTrajectory(self.robot, range(len(self.transferPath)), self.transferPath)
                xferobj = xfertraj.getLinkTrajectory(self.hand.link, resolution)
                xferobj.postTransform(self.Tgrasp)
                # offset times to be just during the transfer stage
                for i in range(len(xferobj.times)):
                    xferobj.times[i] += len(self.transitPath)
                self.objectPath = xferobj
                vis.animate(("world", self.robot.getName()), self.path, endBehavior='halt')
                vis.animate(("world", self.object.getName()), self.objectPath, endBehavior='halt')
            else:
                vis.animate(("world", self.robot.getName()), None, endBehavior='halt')
                vis.animate(("world", self.object.getName()), None, endBehavior='halt')

        if key == 'n':
            print("Moving to next action")
            if self.transitPath and self.transferPath and self.retractPath:
                # update start state
                self.qstart = self.retractPath[-1]
                self.Tstart = self.Tgoal
                self.robot.setConfig(self.qstart)
                self.object.setTransform(*self.Tstart)
                self.transitPath = None
                self.transferPath = None
                # self.hand = None
                self.Tgrasp = None
                self.refresh()
                vis.animate(("world", self.robot.getName()), None, endBehavior='halt')
                vis.animate(("world", self.object.getName()), None, endBehavior='halt')

        if key == 's':
            vis.animate(("world", self.robot.getName()), self.path, endBehavior='halt')
            vis.animate(("world", self.object.getName()), self.objectPath, endBehavior='halt')

        if key == 'o':
            configuration = self.hand.open(self.robot.getConfig(), 1)
            self.robot.setConfig(configuration)
            self.path = RobotTrajectory(self.robot, [1], configuration)
            vis.animate(("world", self.robot.getName()), self.path, endBehavior='halt')
            return True

        if key == 'p':
            configuration = self.hand.open(self.robot.getConfig(), 0)
            self.robot.setConfig(configuration)
            self.path = RobotTrajectory(self.robot, [1], configuration)
            vis.animate(("world", self.robot.getName()), self.path, endBehavior='halt')
            return True
        return False


class TransferCSpace(CSpace):
    def __init__(self, globals, hand, object):
        CSpace.__init__(self)
        self.globals = globals
        self.robot = globals.robot
        self.hand = hand
        self.object = object
        # initial whole-body configuratoin
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
        # test robot-object collisions
        for o in range(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index, o)):
                if o != 0:
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

    print("Trying to find pregrasp config...")
    qpregrasp, qpregrasparm = hand.ikSolver(robot, cspace, obj.getTransform()[1])

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


def planTransfer(world, objectIndex, hand, shift):
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
    print(q0arm, "q0arm")
    if not cspace.feasible(q0arm):
        print("Warning, arm start configuration is infeasible")
        cspace.close()
        return None

    qungrasp = None
    qungrasparm = None
    qungrasp, qungrasparm = hand.ikSolver(robot, cspace,
                                          vectorops.add(vectorops.add(obj.getTransform()[1], [0, 0, 0.3]), shift))

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


def planStraighten(world, objectIndex, hand):
    globals = Globals(world)
    obj = world.rigidObject(objectIndex)
    cspace = TransferCSpace(globals, hand, obj)
    robot = world.robot(0)

    # get the start config
    q0 = robot.getConfig()
    q0arm = [q0[i] for i in hand.armIndices]
    print(q0arm, "q0arm")
    if not cspace.feasible(q0arm):
        print("Warning, arm start configuration is infeasible")
        cspace.close()
        return None

    return


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
    if not cspace.feasible(qtargetarm):
        print("Warning, arm goal configuration is infeasible")

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


if __name__ == "__main__":
    world = WorldModel()
    res = world.readFile("../Python/exercises/manipulation/grasp_attempt.xml")
    if not res: raise RuntimeError("Unable to load world file")
    vis.add("world", world)
    vis.setWindowTitle("Pick and place test, use a/b/c/d to select target")
    vis.pushPlugin(GLPickAndPlacePlugin(world))
    vis.show()
    while vis.shown():
        time.sleep(0.1)
    vis.setPlugin(None)
    vis.kill()

