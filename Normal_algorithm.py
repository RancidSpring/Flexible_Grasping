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
import pyvista as pv
import networkx as nx
from collision_color import *
import itertools
from MeshGeometry import FaceGeom
from Hand import Hand
from TransferUtility import TransferCSpace
from TransitUtility import TransitCSpace
from TransformFunc import *
from Motionplanning import *


class Globals:
    def __init__(self, world):
        self.world = world
        self.robot = world.robot(0)
        self.collider = WorldCollider(world)


class GLPickAndPlacePlugin(GLPluginInterface):
    def __init__(self, world):

        GLPluginInterface.__init__(self)
        self.world = world

        # initialize the kuka arm
        self.hand = Hand(self.world, 'kuka')
        self.trimeshes = self.upload_constraints()
        self.robot = world.robot(0)

        # start robot config
        self.qstart = self.robot.getConfig()

        if len(self.trimeshes) > 0:
            self.object = world.rigidObject(0)

            # start object transform
            self.Tstart = self.object.getTransform()

            # initialize the trimesh, that's currently being evaluated
            self.hand.current_trimesh = self.trimeshes[self.object.index]
        else:
            self.object = None
            # self.collis_object = None
            self.Tstart = None
            self.Tstart_col = None

        # grasp transformation for object and collision area
        self.Tgrasp = None

        # end configuration of the object and collision area
        self.Tgoal = None

        # initialize solution to planning problem
        self.transitPath = None
        self.transferPath = None
        self.retractPath = None
        self.objectPath = None
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
        #glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[3]))
        glColor3f(0, 0, 1)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[1]))
        #glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[4]))
        glColor3f(1, 0, 0)
        glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[2]))
        #glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[5]))

        # glColor3f(0, 0, 1)
        # glVertex3fv(kukaarm.world_pos[0])
        # glColor3f(0, 1, 0)
        # glVertex3fv(kukaarm.world_pos[1])
        # glColor3f(1, 0, 0)
        # glVertex3fv(kukaarm.world_pos[2])
        glColor3f(0.5, 0.5, 0.5)
        for i in range(3):
            glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.GripperPoint1s[i]))
            glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.GripperPoint2s[i]))

        for i in self.hand.faces_geom:
            glVertex3fv(i.end)

        glColor3f(1, 0, 0.5)
        # for i in self.hand.faces_geom:
        #     glVertex3fv(i.face_center)
        for i in self.hand.world_point1s:
            glVertex3fv(i)

        for j in self.hand.world_point2s:
            glVertex3fv(j)

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
                self.robot.setConfig(self.qstart)
                self.object.setTransform(*self.Tstart)
                self.transitPath = None
                self.transferPath = None
                self.Tgrasp = None
                self.hand.refresh_world_coord()
                self.refresh()

        if key == '1':
            self.change_object(0)
        if key == '2':
            self.change_object(1)
        if key == '3':
            self.change_object(2)
        if key == '4':
            self.change_object(3)

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
                   break
            else:
                break
        return True

    def transit_uti(self):
        self.Tgrasp = graspTransform(self.robot, self.hand, self.transitPath[-1], self.Tstart)
        self.robot.setConfig(self.transitPath[-1])
        return 0

    def transfer_uti(self):
        self.Tgoal = graspedObjectTransform(self.robot, self.hand, self.transferPath[0], self.Tstart,
                                            self.transferPath[-1])
        self.robot.setConfig(self.transferPath[-1])
        self.object.setTransform(*self.Tgoal)
        return 0

    def build_object_trajectory(self, resolution=0.05):
        xfertraj = RobotTrajectory(self.robot, range(len(self.transferPath)), self.transferPath)
        xferobj = xfertraj.getLinkTrajectory(self.hand.link, resolution)
        xferobj.postTransform(self.Tgrasp)

        # offset times to be just during the transfer stage
        for i in range(len(xferobj.times)):
            xferobj.times[i] += len(self.transitPath)

        self.objectPath = xferobj
        return 0

    def task_animation(self):
        vis.animate(("world", self.robot.getName()), self.path, endBehavior='halt')
        vis.animate(("world", self.object.getName()), self.objectPath, endBehavior='halt')
        return 0

    def animate_none(self):
        vis.animate(("world", self.robot.getName()), None, endBehavior='halt')
        vis.animate(("world", self.object.getName()), None, endBehavior='halt')

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
        print("Building " + str(constraints_num) + " objects")
        for i in range(constraints_num):
            print("Object number " + str(i))
            print("Choose the object to upload:")
            print("1. Donut with more than one hole")
            print("2. Cursed Cube")
            print("3. Flashlight")
            print("4. Corona Beer")
            print("5. Poopie")
            object_num = int(input())
            if object_num == 1:
                way = "objects/objects/donut.obj"
                scale = [1, 1, 1]
            elif object_num == 2:
                way = "objects/objects/cursed_cube.obj"
                scale = [0.04, 0.04, 0.04]
            elif object_num == 3:
                way = "objects/objects/Flashlight.obj"
                scale = [0.03, 0.03, 0.03]
            elif object_num == 4:
                way = "objects/objects/corona.obj"
                scale = [0.003, 0.003, 0.003]
            elif object_num == 5:
                way = "objects/objects/poopie.obj"
                scale = [0.03, 0.03, 0.03]
            print(
                "Choose the position of the object (type point in world coordinates ex. [1, 1, 1] or any letter to use prepared ones)")
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
                    dest = [0.5, i + 0.15, 0.27]

            own_mesh = trimesh.exchange.load.load(way)
            filled = trimesh.repair.fill_holes(own_mesh)
            third_coloumn = np.multiply(3, np.ones((len(own_mesh.faces), 1)))
            vertices = own_mesh.vertices
            faces = np.hstack((third_coloumn, own_mesh.faces))
            faces = np.hstack(faces).astype(np.int16)
            surf = pv.PolyData(vertices, faces)
            upgraded_mesh = pv.PolyData.subdivide(surf, 1, subfilter='linear')

            faces = upgraded_mesh.faces
            verts = upgraded_mesh.points.astype(np.double)

            for i in range(len(upgraded_mesh.faces)//4):
                faces = np.delete(faces, 3*i)

            m = TriangleMesh()
            for j in np.ndarray.flatten(verts):
                m.vertices.append(j)

            for j in np.ndarray.flatten(faces):
                m.indices.append(int(j))


            new_mesh = world.loadRigidObject("objects/objects/block.obj")
            new_mesh.setName("my_mesh" + str(i))
            new_mesh.geometry().setTriangleMesh(m)
            R = [0, 1, 0, 0, 0, 1, 1, 0, 0]
            rot_matrix = so3.matrix(R)
            new_mesh.setTransform(R, dest)
            new_mesh.geometry().scale(scale[0], scale[1], scale[2])

            color_arr = fill_colors(new_mesh, 10)
            trimeshes.append(own_mesh)
            verts = np.array(m.vertices).reshape(len(m.vertices) // 3, 3)
            verti_arr = []

            for vert in verts:
                vert = np.matmul(rot_matrix, vert)
                vert = np.add(np.multiply(vert, scale), dest)
                verti_arr.append(vert)

            indeces = np.array(m.indices, dtype=np.int32).reshape((len(m.indices) // 3, 3))
            for ind in indeces:
                current_normal = calculate_normal(verti_arr[ind[0]], verti_arr[ind[1]], verti_arr[ind[2]])
                center_of_face = np.mean([verti_arr[ind[0]], verti_arr[ind[1]], verti_arr[ind[2]]], axis=0)
                face_geom = FaceGeom(current_normal, center_of_face)
                self.hand.faces_geom.append(face_geom)

            self.hand.object = world.rigidObject(0)
            self.hand.init_world_pos()

        vis.lock()
        vis.remove("world")
        vis.add("world", self.world)
        vis.colorize.colorize(new_mesh, color_arr)
        #vis.colorize.colorize(new_mesh, 'index', 'random', 'faces')
        vis.unlock()
        return trimeshes


if __name__ == "__main__":
    world = WorldModel()
    collider = WorldCollider(world)

    res = world.readFile("worlds/grasp_attempt.xml")
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