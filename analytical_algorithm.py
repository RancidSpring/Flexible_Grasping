# import klampt modules
from klampt.model import contact
from klampt.model.create import *
from klampt.model.collide import WorldCollider
from klampt.model.trajectory import RobotTrajectory
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from klampt import vis
from klampt.math import vectorops, so3, se3

# import other support packages and libraries
from OpenGL.GL import *
import time
import numpy as np
from numpy import asarray
import trimesh
import pyvista as pv
import itertools
import heapq
import json
import collections

# import own utilities
from utilities.collision_color import *
from utilities.MeshGeometry import FaceGeom
from utilities.TransformFunc import *
from utilities.Motionplanning import *
from Tk_Wrapper import TkWrapper



class Globals:
    def __init__(self, world):
        self.world = world
        self.robot = world.robot(0)
        self.collider = WorldCollider(world)


class GLPickAndPlacePlugin(GLPluginInterface):
    def __init__(self, world, wrapper):

        GLPluginInterface.__init__(self)
        self.world = world
        self.tk_wrapper = wrapper

        # initialize the kuka arm
        self.hand = Hand(self.world, 'kuka')
        self.trimeshes = []
        self.upload_constraints_GUI()
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
            self.Tstart = None

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
        self.force_denied = 0

    def display(self):
        # draw points on the robot
        kukaarm = self.hand
        glDisable(GL_LIGHTING)
        glPointSize(5.0)
        glDisable(GL_DEPTH_TEST)
        glBegin(GL_POINTS)

        # points connected by IKSolver

        # transfer points
        # glColor3f(0, 1, 0)
        # glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[0]))
        # glColor3f(0, 0, 1)
        # glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[1]))
        # glColor3f(1, 0, 0)
        # glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[2]))
        #
        # glColor3f(0, 0, 1)
        # glVertex3fv(kukaarm.transfer_points[0])
        # glColor3f(0, 1, 0)
        # glVertex3fv(kukaarm.transfer_points[1])
        # glColor3f(1, 0, 0)
        # glVertex3fv(kukaarm.transfer_points[2])

        # transit points
        try:
            glColor3f(1, 1, 1)
            glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[3]))
            glColor3f(1, 1, 1)
            glVertex3fv(se3.apply(self.world.robot(0).link(kukaarm.link).getTransform(), kukaarm.local_pos[4]))


            glColor3f(1, 1, 1)
            glVertex3fv(kukaarm.transit_points[0])
            glColor3f(1, 1, 1)
            glVertex3fv(kukaarm.transit_points[1])
        except:
            pass

        # for i in self.hand.faces_geom:
        #     glVertex3fv(i.end)

        # for i in self.world_points:
        #     glVertex3fv(i)

        glColor3f(1, 0, 0.5)
        # for i in self.hand.faces_geom:
        #     glVertex3fv(i.face_center)
        # for i in self.hand.world_point1s:
        #     glVertex3fv(i)

        # for j in self.hand.world_point2s:
        #     glVertex3fv(j)

        # points on the terrain
        # glColor3f(0.5, 0.3, 0)
        # glVertex3fv([0.5, 0.2, 0.27])
        # glColor3f(0.5, 0.3, 1)
        # glVertex3fv([0.5, -0.05, 0.27])
        # glColor3f(0.5, 1, 0.5)
        # glVertex3fv([-0.6, -0.05, 0.27])
        # glColor3f(0.5, 0.3, 0)
        # glVertex3fv([-0.6, 0.2, 0.27])
        glEnd()
        glEnable(GL_DEPTH_TEST)

    def keyboardfunc(self, key, x, y):
        global goal_config_transit
        global goal_rotation_transfer
        h = 0.27
        targets = {'a': [0.5, 0.2, h], 'b': [0.5, -0.05, h], 'c': [-0.6, -0.05, h], 'd': [-0.6, 0, h], 'e': [-0.7, 0.2, h], 'f': [-0.7, 0.1, h], 'g': [-0.6, 0.1, h]}
        if key in targets:
            dest = targets[key]
            return self.planTask(dest, start_rot=False)

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
                self.hand.refresh_world_points()
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
            vis.animate(("world", self.robot.getName()), self.path, endBehavior='halt')
            vis.animate(("world", self.object.getName()), self.objectPath, endBehavior='halt')

        if key == "o":
            for i in range(1, 10):
                qarm_tmp = self.hand.open(self.robot.getConfig(), 1 - (i / 100))
                final_qarm = qarm_tmp[-1]
                self.robot.setConfig(final_qarm)
                if any(globals.collider.robotObjectCollisions(self.robot.index, self.object.index)):
                    break

    # def planTask(self, dest, start_rot):
    #     global goal_config_transit
    #     global goal_rotation_transfer
    #     goal_config_transit = None
    #     goal_rotation_transfer = None
    #     while True:
    #         # set object's starting position
    #         self.object.setTransform(*self.Tstart)
    #
    #         # set robot's starting configuration
    #         self.transitPath = None
    #         self.robot.setConfig(self.qstart)
    #         # self.hand.refresh_world_coord()
    #         self.hand.refresh_world_points()
    #
    #         # plan transit path to grasp object
    #         self.transitPath = planTransit(self.world, self.object.index, self.hand)
    #         if self.transitPath is not None:
    #             if self.transitPath is False:
    #                 break
    #             self.transit_uti()
    #             if self.force_closure():
    #                 for pos in self.hand.world_points:
    #                     data_arr = vectorops.sub(pos, dest)
    #                     self.data.append(data_arr)
    #             else:
    #                break
    #         else:
    #             break
    #     return True

    def planTask(self, dest, start_rot):
        global trans_ind
        global goal_rotation_transfer
        goal_rotation_transfer = None
        trans_ind = 0
        self.Tstart = self.object.getTransform()
        shift = vectorops.sub(dest, self.Tstart[1])
        iteration = 0
        while True:
            # set object's starting position
            self.object.setTransform(*self.Tstart)

            # set robot's starting configuration
            self.transitPath = None
            self.transferPath = None
            self.retractPath = None
            self.robot.setConfig(self.qstart)
            self.hand.refresh_world_points()
            # plan transit path to grasp object
            self.transitPath = planTransit(self.world, self.object.index, self.hand)
            if self.transitPath:
                self.transit_uti()
                # obstacle_queue = obstacle_query(self.object)
                # for i in range(len(obstacle_queue)):
                #     if i < 23:
                #         print(obstacle_queue[i].d, asarray(obstacle_queue[i].cp1), asarray(obstacle_queue[i].cp2), world.terrain(i).getName())
                # plan transfer path with rigidly grasped object
                self.transferPath = planTransfer(self.world, self.object.index, self.hand, shift)
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
            if iteration > 10:
                break

        self.animate_none()
        return False

    def transit_uti(self):
        # color_arr = fill_colors_neutral(self.hand.object, self.hand.grasped_face1, self.hand.grasped_face2)
        # vis.lock()
        # vis.remove("world")
        # vis.add("world", self.world)
        # vis.colorize.colorize(self.hand.object, color_arr)
        # vis.unlock()

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
        try:
            self.object = world.rigidObject(obj_index)
            self.Tstart = self.object.getTransform()
            self.hand.change_the_goal(obj_index)
            self.hand.refresh_world_points()

            self.compute_mesh_geometry(obj_index)
        except:
            print("Can't change the object: object with such index doesn't exist")
            pass
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

    def upload_constraints(self):
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
            print("6. Bulb")
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
            elif object_num == 6:
                way = "objects/objects/bulb.obj"
                scale = [0.05, 0.05, 0.05]
            elif object_num == 7:
                way = "objects/objects/Bulb Pack/Bulbs.obj"
                scale = [0.05, 0.05, 0.05]
            elif object_num == 8:
                # way = "objects/objects/Light Bulb/Light Bulb.obj"
                way = "robots/kuka/links_models/arm_links/link_3.off"
                # scale = [0.05, 0.05, 0.05]
                scale = [1, 1, 1]

            elif object_num == 9:
                way = "robots/kuka/links_models/gripper3fing/gripper_body_smol#1.off"
                scale = [1, 1, 1]
            print("Choose the position of the object (type point in world coordinates ex. [1, 1, 1] or any letter to use prepared ones)")
            position_inp = input()
            if type(position_inp) is str:
                try:
                    dest = json.loads(position_inp)
                except json.decoder.JSONDecodeError:
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
            # filled = trimesh.repair.fill_holes(own_mesh)
            third_coloumn = np.multiply(3, np.ones((len(own_mesh.faces), 1)))
            vertices = own_mesh.vertices
            faces = np.hstack((third_coloumn, own_mesh.faces))
            faces = np.hstack(faces).astype(np.int16)
            surf = pv.PolyData(vertices, faces)
            print("There are ", len(own_mesh.faces), "faces in this object's mesh")
            print("Additional mesh upgrade:")
            print("1. Increase faces number")
            print("2. Reduce faces number")
            print("3. Leave the object as it is")
            try:
                face_mode = int(input())
            except ValueError:
                face_mode = 3

            if face_mode == 1:
                print("How much new faces have to be created?")
                print("1. Coefficient of subdivision is 1. Each face is divided into 4 new ones")
                print("2. Coefficient of subdivision is 2. Each face is divided into 16 new ones")
                try:
                    subdivision_coeff = int(input())
                except ValueError:
                    print("Wrong input type, input has to be integer")
                    print("Leaving mesh as it is")
                    subdivision_coeff = 0
                upgraded_mesh = pv.PolyData.subdivide(surf, subdivision_coeff, subfilter='linear')

            elif face_mode == 2:
                print("Enter float in range [0, 1] which is how much the mesh is reduced in per cent")
                try:
                    decimate_coeff = float(input())
                except ValueError:
                    print("Wrong input type, input has to be float")
                    print("Leaving mesh as it is")
                    decimate_coeff = 0
                if decimate_coeff < 0 or decimate_coeff > 1:
                    print("Wrong range of the input")
                    print("Leaving mesh as it is")
                    decimate_coeff = 0
                upgraded_mesh = surf.decimate(target_reduction=decimate_coeff, volume_preservation=True)

            else:
                upgraded_mesh = surf

            faces = upgraded_mesh.faces
            verts = upgraded_mesh.points.astype(np.double)

            for k in range(len(upgraded_mesh.faces)//4):
                faces = np.delete(faces, 3*k)

            m = TriangleMesh()
            for j in np.ndarray.flatten(verts):
                m.vertices.append(j)

            print("There are ", len(np.ndarray.flatten(faces)), " faces in this mesh")
            for j in np.ndarray.flatten(faces):
                m.indices.append(int(j))

            new_mesh = world.loadRigidObject("objects/objects/block.obj")
            new_mesh.setName("my_mesh" + str(i))
            new_mesh.geometry().setTriangleMesh(m)
            R = [0, 1, 0, 0, 0, 1, 1, 0, 0]
            new_mesh.setTransform(R, dest)
            new_mesh.geometry().scale(scale[0], scale[1], scale[2])

            # color_arr = fill_colors(new_mesh, 10)
            self.trimeshes.append((m, scale))

            self.compute_mesh_geometry(new_mesh.index) # compute normal vectors and create an array containing Face_Geom

            self.hand.object = world.rigidObject(i)
            self.hand.init_transfer_points()
            self.hand.init_transit_points()

        vis.lock()
        vis.remove("world")
        vis.add("world", self.world)
        #vis.colorize.colorize(new_mesh, color_arr)
        for i in range(len(self.trimeshes)):
            vis.colorize.colorize(world.rigidObject(i), 'index', 'random', 'faces')
        vis.unlock()
        return self.trimeshes

    def upload_constraints_GUI(self):
        i = 0
        for obj in self.tk_wrapper.object_load:
            way = obj.way
            scale = obj.scale
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

            if obj.idx == 6:
                dest[2] += 0.03

            own_mesh = trimesh.exchange.load.load(way, force="mesh")
            # own_mesh = own_mesh.convex_hull
            faces = own_mesh.faces
            third_coloumn = np.multiply(3, np.ones((len(faces), 1)))
            vertices = own_mesh.vertices
            faces = np.hstack((third_coloumn, faces))
            faces = np.hstack(faces).astype(np.int16)
            surf = pv.PolyData(vertices, faces)
            if obj.upgrade_coef == 1:
                upgraded_mesh = pv.PolyData.subdivide(surf, obj.subdivision_coeff, subfilter='linear')

            elif obj.upgrade_coef == 2:
                upgraded_mesh = surf.decimate(target_reduction=obj.decimate_coeff, volume_preservation=True)

            else:
                upgraded_mesh = surf

            faces = upgraded_mesh.faces
            verts = upgraded_mesh.points.astype(np.double)

            for k in range(len(upgraded_mesh.faces)//4):
                faces = np.delete(faces, 3*k)

            m = TriangleMesh()
            for j in np.ndarray.flatten(verts):
                m.vertices.append(j)

            for j in np.ndarray.flatten(faces):
                m.indices.append(int(j))

            new_mesh = world.loadRigidObject("objects/objects/block.obj")
            new_mesh.setName("my_mesh" + str(i))
            new_mesh.geometry().setTriangleMesh(m)
            R = [0, 1, 0, 0, 0, 1, 1, 0, 0]
            new_mesh.setTransform(R, dest)
            new_mesh.geometry().scale(scale[0], scale[1], scale[2])

            # color_arr = fill_colors(new_mesh, 10)
            self.trimeshes.append((m, scale))

            self.compute_mesh_geometry(new_mesh.index) # compute normal vectors and create an array containing Face_Geom

            self.hand.object = world.rigidObject(i)
            self.hand.init_transfer_points()
            self.hand.init_transit_points()
            i += 1
            vis.lock()
            vis.remove("world")
            vis.add("world", self.world)
            # if obj.colors is not None:
            #     color_arr = self.sort_color_array(obj.colors, own_mesh.faces)
            #     vis.colorize.colorize(new_mesh, color_arr)
            # else:
            for i in range(len(self.trimeshes)):
                vis.colorize.colorize(new_mesh, 'index', 'random', 'faces')
            vis.unlock()
        return self.trimeshes

    def compute_mesh_geometry(self, obj_ind):
        start = time.time()
        self.hand.faces_geom = []
        m, scale = self.trimeshes[obj_ind]
        R = world.rigidObject(obj_ind).getTransform()[0]
        rot_matrix = so3.matrix(R)
        dest = world.rigidObject(obj_ind).getTransform()[1]
        verts = np.array(m.vertices, dtype=np.double).reshape(len(m.vertices) // 3, 3)
        verti_arr = []

        for vert in verts:
            vert = np.matmul(rot_matrix, vert)
            vert = np.add(np.multiply(vert, scale), dest)
            verti_arr.append(vert)

        indeces = np.array(m.indices, dtype=np.int32).reshape((len(m.indices) // 3, 3))
        index_cnt = 0
        for ind in indeces:
            current_normal = calculate_normal(verti_arr[ind[0]], verti_arr[ind[1]], verti_arr[ind[2]])
            center_of_face = np.mean([verti_arr[ind[0]], verti_arr[ind[1]], verti_arr[ind[2]]], axis=0)
            face_geom = FaceGeom(current_normal, center_of_face, index_cnt)
            self.hand.faces_geom.append(face_geom)
            index_cnt += 1
        end = time.time()
        print("Computing vertices and normals time", end-start)
        self.create_heap()

    def create_heap(self):
        start = time.time()
        self.hand.heap = []
        cnt = 0
        for n1, n2 in itertools.combinations(self.hand.faces_geom, 2):
            cnt += 1
            dot_product = np.dot(n1.normal_vec, n2.normal_vec)
            if dot_product >= 0:
                continue
            cross_product = np.cross(n1.normal_vec, n2.normal_vec)
            # print("CROSS PRODUCT", cross_product)
            cross_norm = np.linalg.norm(cross_product)
            if isclose(cross_norm, 0, abs_tol=1e-9):
                center_of_mass = np.multiply(np.add(n1.face_center, n2.face_center), 0.5)
                distance = np.linalg.norm(np.subtract(n2.face_center, n1.face_center))
                self.hand.heap.append((distance, (list(center_of_mass), n1, n2)))  # add pairs to heap
        heapq.heapify(self.hand.heap)  # heap sort by minimum distance
        print("CNT ITERATIONS", cnt)
        end = time.time()
        print("Create Heap time", end-start)
        print("Heap length", len(self.hand.heap))
        return

    # def sort_color_array(self, color_arr_tmp, sorted_array):
    #     sorted_array = sorted_array.view(np.ndarray)
    #     color_arr = [[0 for col in range(4)] for row in range(len(sorted_array))]
    #
    #     new_sorted = []
    #     for subarr in sorted_array:
    #         newsubarr = sorted(subarr)
    #         new_sorted.append(newsubarr)
    #
    #     unpack = []
    #     for subarr in color_arr_tmp:
    #         newsubarr = sorted(subarr[0])
    #         unpack.append(newsubarr)
    #
    #     # new_sorted = np.array(sorted_array)
    #     # unpack = np.array(unpack)
    #
    #     for k in range(len(unpack)):
    #         index = new_sorted.index(unpack[k])
    #         color_arr[index][0] = color_arr_tmp[k][1][0]
    #         color_arr[index][1] = color_arr_tmp[k][1][1]
    #         color_arr[index][2] = color_arr_tmp[k][1][2]
    #         color_arr[index][3] = color_arr_tmp[k][1][3]
    #     print(color_arr)
    #     return color_arr

if __name__ == "__main__":
    world = WorldModel()
    collider = WorldCollider(world)
    wrapp = TkWrapper()
    wrapp.main_func()

    gripper = wrapp.gripper_type
    if gripper == "2fing":
        from adaptive_2fing.Hand import Hand
        world_way = "worlds/grasp_attempt.xml"
    elif gripper == "3fing_parallel":
        from parallel_3fing.Hand import Hand
        world_way = "worlds/grasp_attempt3fing_parallel.xml"
    elif gripper == "3fing_adaptive":
        from adaptive_3fing.Hand import Hand
        world_way = "worlds/grasp_attempt3fing_adaptive.xml"

    res = world.readFile(world_way)
    if not res: raise RuntimeError("Unable to load world file")
    vis.add("world", world)
    vis.setWindowTitle("Pick and place test, use a/b/c/d to select target")
    vis.pushPlugin(GLPickAndPlacePlugin(world, wrapp))

    # visualize fingers for 2 finger robot
    # vis.setColor(('world', 'kuka', world.robot(0).link(17).getName()), 0, 255, 0, a=1.0)
    # vis.setColor(('world', 'kuka', world.robot(0).link(19).getName()), 0, 255, 0, a=1.0)

    # visualize fingers for 3 finger adaptive gripper on the robot
    # vis.setColor(('world', 'kuka', world.robot(0).link(27).getName()), 0, 255, 0, a=1.0)
    # vis.setColor(('world', 'kuka', world.robot(0).link(21).getName()), 0, 255, 0, a=1.0)
    # vis.setColor(('world', 'kuka', world.robot(0).link(24).getName()), 0, 255, 0, a=1.0)

    # visualize fingers for 3 finger parallel gripper on the robot
    # vis.setColor(('world', 'kuka', world.robot(0).link(17).getName()), 0, 255, 0, a=1.0)
    # vis.setColor(('world', 'kuka', world.robot(0).link(16).getName()), 0, 255, 0, a=1.0)
    # vis.setColor(('world', 'kuka', world.robot(0).link(15).getName()), 0, 255, 0, a=1.0)
    vis.show()

    while vis.shown():
        time.sleep(0.1)
    vis.setPlugin(None)
    vis.kill()