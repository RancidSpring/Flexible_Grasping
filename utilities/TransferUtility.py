from klampt import *
from klampt.model.create import *
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from klampt import vis
from klampt.math import vectorops, so3, se3
from klampt.plan.cspace import CSpace, MotionPlan
from OpenGL.GL import *
from utilities.collision_color import *
from utilities.TransformFunc import *


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
                    # print("ROBOT-OBJECT COLLISION", o, grasped_object)
                    return False

        # test object-object collisions
        for o in range(world.numRigidObjects()):
            if any(collider.objectObjectCollisions(o, None)):
                # print("OBJECT-OBJECT COLLISION")
                return False

        # if collider.objectObjectCollisions(0, 2):
        #     print("TUT")
        #     return False

        # test object-terrain collisions
        for o in range(world.numRigidObjects()):
            if any(collider.objectTerrainCollisions(o, None)):
                # print("OBJECT-TERRAIN COLLISION")
                return False

        # test robot-terrain collisions
        for o in range(world.numTerrains()):
            if any(collider.robotTerrainCollisions(self.robot.index, o)):
                # print("ROBOT-TERRAIN COLLISION")
                return False

        # test robot self-collisions
        if any(collider.robotSelfCollisions(self.robot.index)):
            collision_link1 = list(collider.robotSelfCollisions(self.robot.index))[0][0].getName()
            collision_link2 = list(collider.robotSelfCollisions(self.robot.index))[0][1].getName()
            vis.setColor(('world', 'kuka', collision_link1), 255, 0, 0, a=1.0)
            vis.setColor(('world', 'kuka', collision_link2), 255, 0, 0, a=1.0)
            return False
        return True
