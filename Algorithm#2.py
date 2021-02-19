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