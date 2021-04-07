from klampt import *
from klampt.model.create import *
from klampt.vis.glcommon import *
from klampt.vis.colorize import *
from OpenGL.GL import *


def fill_colors_collision(object, face_number1, face_number2 = None):
    """ Function that colorize special faces of the object base on the face_number parameter"""
    num_faces = object.geometry().numElements()
    green = [0.0, 1.0, 0.0]
    red = [1.0, 0.0, 0.0]
    color_arr = []
    for i in range(num_faces):
        if face_number2 is not None:
            if i == face_number1 or i == face_number2:
                rgb = red
            else:
                rgb = green
            color_arr.append(rgb)
        else:
            if i == face_number1:
                rgb = red
            else:
                rgb = green
            color_arr.append(rgb)
    print(color_arr)
    return color_arr


def fill_colors_neutral(object, face_number1, face_number2=None):
    """ Function that colorize special faces of the object base on the face_number parameter"""
    num_faces = object.geometry().numElements()
    green = [0.0, 1.0, 0.0]
    red = [1.0, 0.0, 0.0]
    blue = [0, 0, 1.0]
    color_arr = []
    for i in range(num_faces):
        if face_number2 is not None:
            if i == face_number1 or i == face_number2:
                rgb = green
            else:
                rgb = blue
            color_arr.append(rgb)
        else:
            if i == face_number1:
                rgb = green
            else:
                rgb = blue
            color_arr.append(rgb)
    print(color_arr)
    return color_arr