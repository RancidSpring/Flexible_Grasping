import bpy
import numpy as np

file_loc = '../../CIIRC/Klampt/Klampt-examples/Grasping/Flexible_Grasping/objects/objects/Flashlight2.obj'
imported_object = bpy.ops.import_scene.obj(filepath=file_loc)
obj_object = bpy.context.selected_objects[0] ####<--Fix
print('Imported name: ', obj_object.name)
mesh = obj_object.data

color_array = []

for f in mesh.polygons:  # iterate over faces
    print("face", f.index, "material_index", f.material_index)
    slot = obj_object.material_slots[f.material_index]
    mat = slot.material
    if mat is not None:
        print(mat.name)
        print(list(mat.diffuse_color))
        color_array.append(list(mat.diffuse_color))
    else:
        print("No mat in slot", f.material_index)
        
save_directory = '../../CIIRC/Klampt/Klampt-examples/Grasping/Flexible_Grasping/objects/objects/colors/flashlight_color_array.npy'
print(color_array)
np.save(save_directory, color_array)