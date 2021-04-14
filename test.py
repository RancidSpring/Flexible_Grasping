import trimesh
# import pyvista as pv
import numpy as np
from Py3DViewer import Trimesh
import time
from pygel3d import hmesh, gl_display as gl
from pygel3d import jupyter_display as jd
from tkinter import *
import os
import sys, string, os


def plot_subdivisions(mesh, a, b):
    display_args = dict(show_edges=True, color=True)
    p = pv.Plotter(shape=(3,3))

    for i in range(3):
        p.subplot(i,0)
        p.add_mesh(mesh, **display_args)
        p.add_text("Original Mesh")

    def row_plot(row, subfilter):
        subs = [a, b]
        for i in range(2):
            p.subplot(row, i+1)
            p.add_mesh(pv.PolyData.subdivide(mesh, subs[i], subfilter='linear'), **display_args)
            p.add_text(f"{subfilter} subdivision of {subs[i]}")
    row_plot(0, "linear")
    row_plot(1, "butterfly")
    row_plot(2, "loop")

    p.link_views()
    p.view_isometric()
    return p

# #way = "objects/objects/cursed_cube.obj"
# way = "objects/objects/Flashlight.obj"
#
# own_mesh = trimesh.exchange.load.load(way)
# filled = trimesh.repair.fill_holes(own_mesh)
#
# third_coloumn = np.multiply(3, np.ones((len(own_mesh.faces), 1)))
# vertices = own_mesh.vertices
# faces = np.hstack((third_coloumn, own_mesh.faces))
# faces = np.hstack(faces).astype(np.int16)
# surf = pv.PolyData(vertices, faces)
# plotter = plot_subdivisions(surf, 1, 2)
# new_mesh = pv.PolyData.subdivide(surf, 1, subfilter='linear')
# print("zho", new_mesh.faces)
# print("pa", new_mesh.points)
#
# #cpos = [(-0.02788175062966399, 0.19293295656233056, 0.4334449972621349),
#         # (-0.053260899930287015, 0.08881197167521734, -9.016948161029588e-05),
#         # (-0.10170607813337212, 0.9686438023715356, -0.22668272496584665)]
#
# #plotter.camera_position = cpos
# plotter.show()


# arr = np.array([5, 2, 1, 5])
# index = arr.
# print(lol)



# way = "objects/objects/Flashlight2.obj"
# f = open(way, "r")
# resolver = trimesh.resolvers.FilePathResolver.get(mtl)
# print(dir(resolver))
# own_mesh = trimesh.exchange.obj.load_obj(f, resolver)
# print(dir(trimesh.visual.color.ColorVisuals))
# own_mesh = trimesh.load(way)
#
# # np.set_printoptions(threshold=sys.maxsize)
# visuals = trimesh.visual.color.ColorVisuals(list(own_mesh.geometry.items())[1][1])
# print(visuals.face_colors)

file_name = 'objects/objects/Flashlight2.obj'
# # resolver = trimesh.visual.resolvers.FilePathResolver(file_name)
# # with open(file_name, 'r') as f:
# #     kwargs = trimesh.exchange.obj.load_obj(f, resolver=resolver)
# # print(kwargs)
# # scene = trimesh.exchange.load.load_kwargs(kwargs)
#
#
#
# mesh = trimesh.load(file_name)
# mesh.show()
# visuals = list(mesh.geometry.items())[1][1].visual.to_color().face_colors
# # visuals = mesh.visual
# print(visuals)



# import pywavefront
# scene = pywavefront.Wavefront('objects/objects/Flashlight2.obj')
# print(scene)

# m = hmesh.load("objects/objects/corona.obj")
# viewer = gl.Viewer()
# viewer.display(m, mode='w')
# jd.set_export_mode(True)
# jd.display(m, smooth=False)

start = time.time()
a = [[1, 5, 2], [10, 4, 11], [100, 50, 200]]
b = []
for lol in a:
    kek = sorted(lol)
    b.append(kek)
end = time.time()
print(a)
print(b)
print(end - start)


start = time.time()

a = [[1, 5, 2], [10, 4, 11], [100, 50, 200]]

for j in range(len(a)):
    a[j] = sorted(a[j])
end = time.time()
print(a)
print(b)
print(end-start)

color_arr = [[0 for col in range(3)] for row in range(10)]
color_arr[2][2] = 210
print(color_arr)