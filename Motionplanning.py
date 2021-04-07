from klampt import *
from klampt.model.create import *
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from klampt.model.collide import WorldCollider
from klampt.math import vectorops, so3, se3
from klampt.plan.cspace import CSpace, MotionPlan
from OpenGL.GL import *
import time
from collision_color import *
from TransferUtility import TransferCSpace
from TransitUtility import TransitCSpace



class Globals:
    def __init__(self, world):
        self.world = world
        self.robot = world.robot(0)
        self.collider = WorldCollider(world)


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

    if solution is not None and solution:
        qpregrasp, qpregrasparm = solution
    else:
        return None

    print("Planning transit motion to pregrasp config...")
    MotionPlan.setOptions(connectionThreshold=5.0, perturbationRadius=0.5)
    # algorithm = "lazyrrg*"
    # algorithm = "sbl"
    algorithm = "lazyprm*"

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
        qarm_tmp = path + [hand.open(path[-1], 1-(i/100))]
        final_qarm = qarm_tmp[-1]
        robot.setConfig(final_qarm)
        if any(globals.collider.robotObjectCollisions(robot.index, obj.index)):
            break

    if not cspace.feasible(final_qarm):
        print("Fragile area is involved")
        return None

    # add a path to the grasp configuration
    return qarm_tmp


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
    if not cspace.feasible(q0arm):
        print("Warning, arm start configuration is infeasible")
        return None
    start = time.time()
    solution = hand.ikSolver(robot, cspace,
                                          vectorops.add(vectorops.add(obj.getTransform()[1], [0, 0, 0.0]), shift), True, False)
    end = time.time()
    print("transfer time", end - start)
    if solution is not None:
        qungrasp, qungrasparm = solution
    else:
        return None
    # plan the transfer path between q0arm and qungrasparm
    print("Planning transfer motion to ungrasp config...")
    # algorithm = "lazyrrg*"
    algorithm = "lazyprm*"
    # algorithm = "sbl"

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

    qpostungrasp = hand.open(qungrasp, 1)
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
    #algorithm = "lazyrrg*"
    # algorithm = "sbl"

    algorithm = "lazyprm*"

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
