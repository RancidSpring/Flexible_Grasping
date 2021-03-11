from klampt import *
from klampt.model.create import *
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from OpenGL.GL import *


def fill_colors(object, face_number):
    """ Function that colorize special faces of the object base on the face_number parameter"""
    num_faces = object.geometry().numElements()
    green = [0.0, 1.0, 0.0]
    red = [1.0, 0.0, 0.0]
    color_arr = []
    for i in range(num_faces):
        if i == face_number:
            rgb = red
        else:
            rgb = green
        color_arr.append(rgb)
    print(color_arr)
    return color_arr
