from klampt import *
from klampt.model.create import *
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from klampt import vis
from klampt.plan.cspace import CSpace, MotionPlan
from OpenGL.GL import *
import numpy as np
from collision_color import *


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
                        #print("Contact of the object with the 1st finger is in the index ", cont_elem_obj_fing1[i])
                        if 0 <= cont_elem_obj_fing1[i] < self.hand.object.geometry().numElements():
                            contact_color = self.hand.object.appearance().getElementColor(3, cont_elem_obj_fing1[i])
                            #print("Color of the contact face", contact_color)
                            if np.array_equal([1.0, 0.0, 0.0, 1.0], contact_color):
                                return False

                    cont_elem_obj_fing2 = world.rigidObject(o).geometry().contacts(world.robot(0).link(19).geometry(), 0, 0).elems1
                    for i in range(len(cont_elem_obj_fing2)):
                        #print("Contact of the object with the 2st finger is in the index ", cont_elem_obj_fing2[i])
                        if 0 <= cont_elem_obj_fing2[i] < self.hand.object.geometry().numElements():
                            contact_color = self.hand.object.appearance().getElementColor(3, cont_elem_obj_fing2[i])
                            #print("Color of the contact face", contact_color)
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
