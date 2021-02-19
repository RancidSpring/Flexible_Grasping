import klampt
from klampt.vis import GLRealtimeProgram
from klampt.model.subrobot import RobotModelLink
from klampt import robotsim
from klampt import *
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
    Members include hand (either 'l' or 'r'), link (the hand link index),
    localPosition1 and localPosition2 (two points along the middle axis of the hand)
    armIndices (the hand's arm link indices), and fingerIndices (the hand's
    finger link indices).
    """

    def __init__(self, world, hand='kuka'):
        self.hand = hand
        self.armIndices = range(6, 13)
        self.gripperIndex = 14
        self.link = 14
        self.localPosition1 = (0.01, 0, 0.15)
        self.localPosition2 = (-0.01, 0, 0.15)
        self.world = world
        self.subrobot = model.subrobot.SubRobotModel(self.world.robot(0), ["grip0", "grip1", "grip2", "grip3", "grip4"])
        self.grip_config = self.subrobot.setConfig([0, 1, 0, -1, 0])

    def open(self, q, amount=1.0):
        """Computes a configuration identical to q but with hand open
        if amount = 1 or closed if amount = 1."""
        return openhand(q, self.hand, amount)

    def ikSolver(self, robot, obj_pt, obj_axis):
        """Returns an IK solver that places the hand center at obj_pt with
        the palm oriented along obj_axis"""
        q = robot.getConfig()
        obj = IKObjective()
        print(obj_pt)
        obj_axis = [0, 1, 0]
        obj.setFixedPoints(self.link, [self.localPosition1, self.localPosition2],
                           [vectorops.madd(obj_pt, obj_axis, 0.03), vectorops.madd(obj_pt, obj_axis, -0.05)])
        #obj.setFixedTransform(self.link, [-1, 0, 0, 0, 1, 0, 0, 0, -1], [1, 1, 1])
        solver = IKSolver(robot)
        solver.add(obj)
        solver.setActiveDofs(self.armIndices)
        return solver

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
        print("LLOLOLOLOLOLOLLL")
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
        #test robot-object collisions
        for o in range(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index, o)):
                if o != 0:
                    return False

        # test robot-terrain collisions
        for o in range(world.numTerrains()):
            if any(collider.robotTerrainCollisions(self.robot.index, o)):
                # collision_link1 = list(collider.robotTerrainCollisions(self.robot.index))[0][0].getName()
                # vis.setColor(('world', 'kuka', collision_link1), 255, 0, 0, a=1.0)
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
        self.hand = Hand(self.world, 'kuka')
    def display(self):
        # draw points on the robot
        kukaarm = Hand(self.world, 'kuka')
        glDisable(GL_LIGHTING)
        glPointSize(5.0)
        glDisable(GL_DEPTH_TEST)
        glBegin(GL_POINTS)
        glColor3f(0, 1, 0)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.localPosition1))
        glColor3f(0, 0, 1)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.localPosition2))
        glColor3f(1, 0, 0)
        glVertex3fv(self.world.rigidObject(0).getTransform()[1])
        glColor3f(1, 1, 0)
        glVertex3fv(vectorops.madd(self.world.rigidObject(0).getTransform()[1], [0, 1, 0], 0.03))
        glColor3f(1, 1, 1)
        glVertex3fv(vectorops.madd(self.world.rigidObject(0).getTransform()[1], [0, 1, 0], -0.02))
        glColor3f(0.5, 0.5, 0)
        glVertex3fv([0.5, 0.2, 0.265])
        glColor3f(0.5, 0, 0.8)
        glVertex3fv([0.5, -0.05, 0.265])
        glColor3f(0.5, 0.5, 1)
        glVertex3fv([-0.5, -0.05, 0.265])
        glColor3f(0.5, 1, 0)
        glVertex3fv([-0.5, 0.2, 0.265])
        glEnd()
        glEnable(GL_DEPTH_TEST)

    def keyboardfunc(self, key, x, y):
        for i in range(10):
            file = open("output.txt", "a")
            h = 0.265
            targets = {'a': (0.5, 0.2, h), 'b': (0.5, -0.05, h), 'c': (-0.5, -0.05, h), 'd': (-0.5, 0.2, h)}
            if key in targets:
                start = time.time()
                dest = targets[key]
                shift = vectorops.sub(dest, self.Tstart[1])
                self.object.setTransform(*self.Tstart)
                self.robot.setConfig(self.qstart)
                self.animationTime = 1
                # plan transit path to grasp object
                file.write("TRANSIT \n \n")
                self.transitPath = planTransit(self.world, 0, self.hand, file)
                if self.transitPath:
                    end = time.time()
                    file.write("transitPath construct time:\t" + str(end - start) + "\n \n \n")
                    print("transitPath construct time:\t", end - start)
                    # plan transfer path
                    self.Tgrasp = graspTransform(self.robot, self.hand, self.transitPath[-1], self.Tstart)
                    self.robot.setConfig(self.transitPath[-1])
                    file.write("TRANSFER \n \n")
                    self.transferPath = planTransfer(self.world, 0, self.hand, shift, file)
                    if self.transferPath:
                        self.Tgoal = graspedObjectTransform(self.robot, self.hand, self.transferPath[0], self.Tstart,
                                                            self.transferPath[-1])
                        end = time.time()
                        file.write("transferPath construct time:\t" + str(end - start) + "\n \n \n")
                        print("transferPath construct time:\t", end - start)
                        # plan free path to retract arm
                        self.robot.setConfig(self.transferPath[-1])
                        self.object.setTransform(*self.Tgoal)
                        file.write("FREE \n \n")
                        self.retractPath = planFree(self.world, self.hand, self.qstart, file)
                        end = time.time()
                        file.write("FreePath construct time:\t" + str(end - start) + "\n \n \n ")
                        print("FreePath construct time: ", end - start)
                        file.write("_____________________________________________________" + "\n")
                        file.close()
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
                    vis.animate(("world", self.robot.getName()), self.path)
                    vis.animate(("world", self.object.getName()), self.objectPath)
                else:
                    vis.animate(("world", self.robot.getName()), None)
                    vis.animate(("world", self.object.getName()), None)
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
                    #self.hand = None
                    self.Tgrasp = None
                    self.refresh()
                    vis.animate(("world", self.robot.getName()), None)
                    vis.animate(("world", self.object.getName()), None)

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
        #test robot-object collisions
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

def planTransfer(world, objectIndex, hand, shift, file):
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
        print("TODO: Complete 2.a to bypass this error")
        raw_input()
        cspace.close()
        return None

    # TODO: get the ungrasp config using an IK solver
    qungrasp = None
    qungrasparm = None
    start = time.time()
    # raw_input()
    solver = hand.ikSolver(robot, vectorops.add(obj.getTransform()[1], shift), (0, 0, 1))
    numRestarts = 1000
    for i in range(numRestarts):
        solver.sampleInitial()
        solver.setMaxIters(200)
        solver.setTolerance(1e-3)
        res = solver.solve()
        if res:
            break
    end = time.time()
    file.write("numRestarts IKSOLVER TRANFER\t" + str(numRestarts) + "\n")

    file.write("IKSOLVER TRANFER:\t" + str(end - start) + "\n")
    print("IKSOLVER TRANFER: ", end - start)
    if res:
        qungrasp = robot.getConfig()
        qungrasparm = [qungrasp[i] for i in hand.armIndices]
        if not cspace.feasible(qungrasparm):
            print("Pregrasp config infeasible")
            cspace.close()
            return None
    if qungrasp == None:
        print("Ungrasp solve failed")
        cspace.close()
        return None
    # plan the transfer path between q0arm and qungrasparm
    print("Planning transfer motion to ungrasp config...")
    start = time.time()
    algorithm = "rrt*"
    MotionPlan.setOptions(connectionThreshold=5.0, perturbationRadius=0.5)
    planner = MotionPlan(cspace, algorithm)
    planner.setEndpoints(q0arm, qungrasparm)
    # raw_input()
    iters = 0
    step = 10
    while iters < 1000:
        planner.planMore(step)
        iters += step
    # cspace.close()
    if planner.getPath() == None:
        print("Failed finding transit path")
        return None
    print("Success, found path with", len(planner.getPath()), "milestones")
    end = time.time()

    file.write("MOTIONPLANNING Algorithm:\t" + str(algorithm) + "\n")
    file.write("iterations MOTIONPLANNING TRANSFER:\t" + str(iters) + "\n")
    file.write("MOTIONPLANNING TRANSFER:\t" + str(end - start) + "\n")
    print("MOTIONPLANNING TRANSFER:\t", end - start)
    # cspace.close()
    # lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi, i in zip(qarm, hand.armIndices):
            path[-1][i] = qi

    qpostungrasp = hand.open(qungrasp, 1.0)
    return path + [qpostungrasp]

def planTransit(world, objectIndex, hand, file):
    globals = Globals(world)
    cspace = TransitCSpace(globals, hand)
    obj = world.rigidObject(objectIndex)
    robot = world.robot(0)
    qmin, qmax = robot.getJointLimits()

    # get the start config
    q0 = robot.getConfig()
    q0arm = [q0[i] for i in hand.armIndices]
    if not cspace.feasible(q0arm):
        print("Warning, arm start configuration is infeasible")
    qpregrasp = None
    qpregrasparm = None
    start = time.time()
    solver = hand.ikSolver(robot, obj.getTransform()[1], [0, 0, 1])
    print("Trying to find pregrasp config...")
    numRestarts = 1000
    for i in range(numRestarts):
        solver.sampleInitial()
        solver.setMaxIters(100)
        solver.setTolerance(1e-3)
        res = solver.solve()
        if res:
            break
    end = time.time()
    file.write("numRestarts IKSOLVER TRANSIT\t" + str(numRestarts) + "\n")
    file.write("IKSOLVER TRANSIT:\t" + str(end - start) + "\n")
    print("IKSOLVER TRANSIT:\t", end - start)
    if res:
        qpregrasp = robot.getConfig()
        qpregrasparm = [qpregrasp[i] for i in hand.armIndices]
        if not cspace.feasible(qpregrasparm):
            print("Pregrasp config infeasible")
            cspace.close()
            return None
    if qpregrasp == None:
        print("Pregrasp solve failed")
        cspace.close()
        return None
    start = time.time()
    print("Planning transit motion to pregrasp config...")
    MotionPlan.setOptions(connectionThreshold=5.0, perturbationRadius=0.5)
    algorithm = "rrt*"
    planner = MotionPlan(cspace, algorithm)
    planner.setEndpoints(q0arm, qpregrasparm)
    iters = 0
    step = 10
    while planner.getPath() == None and iters < 1000:
        planner.planMore(step)
        iters += step
    cspace.close()
    if planner.getPath() == None:
        print("Failed finding transit path...")
        return None
    end = time.time()
    file.write("MOTIONPLANNING Algorithm:\t" + str(algorithm) + "\n")
    file.write("iterations MOTIONPLANNING TRANSIT:\t" + str(iters) + "\n")
    file.write("MOTIONPLANNING TRANSIT:\t" + str(end - start) + "\n")
    print("MOTIONPLANNING TRANSIT:  ", end - start)
    print("Success, found path with", len(planner.getPath()), "milestones...")
    # lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi, i in zip(qarm, hand.armIndices):
            path[-1][i] = qi

    # add a path to the grasp configuration
    return path + [hand.open(path[-1], 0)]

def planFree(world, hand, qtarget, file):
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
    start = time.time()
    algorithm = "rrt*"
    MotionPlan.setOptions(connectionThreshold=5.0, perturbationRadius=0.5)
    planner = MotionPlan(cspace, algorithm)
    planner.setEndpoints(q0arm, qtargetarm)
    iters = 0
    step = 10
    while iters < 1000:
        planner.planMore(step)
        iters += step
    cspace.close()
    if planner.getPath() == None:
        print("Failed finding transit path")
        return None
    print("Success")
    end = time.time()
    file.write("MOTIONPLANNING Algorithm:\t" + str(algorithm) + "\n")
    #file.write("numRestarts FREE MOTIONPLANNING: " + str(numRestarts) + "\n")
    file.write("iterations FREE MOTIONPLANNING:\t" + str(iters) + "\n")
    #file.write("iterations " + str(end - start) + "\n")
    file.write("FREE MOTIONPLANNING:\t"+ str(end - start) + "\n")
    print("FREE MOTIONPLANNING:     " + str(end - start))
    # lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi, i in zip(qarm, hand.armIndices):
            path[-1][i] = qi
    return path

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

