import bpy
import bmesh
import os
import math

#reset factory setting and then clearing the data
def reset_blend():
    #bpy.ops.wm.read_factory_settings()

    for scene in bpy.data.scenes:
        for obj in scene.objects:
            scene.objects.unlink(obj)

    # only the data in the startup scene
    for bpy_data_iter in (
            bpy.data.objects,
            bpy.data.meshes,
            bpy.data.lamps,
            bpy.data.cameras,
            bpy.data.textures
    ):
        for id_data in bpy_data_iter:
            bpy_data_iter.remove(id_data)
            
reset_blend()

# put the location to the folder where the .obj files are located here in this fashion
path_to_obj_dir = '/home/pavd/Desktop/Surfaces_ID_Complete_Trial/Flat_blackened_coloured1/'

# get list of all files in directory
file_list = sorted(os.listdir(path_to_obj_dir))

# get a list of files ending in 'obj'
obj_list = [item for item in file_list if item.endswith('.obj')]
print("="*60) 
print("objects: ", obj_list)
print("="*60) 

# loop through the strings in obj_list and add the files to the scene 
# (just in case of multiple .obj files)
for item in obj_list:
    path_to_file = os.path.join(path_to_obj_dir, item)
    bpy.ops.import_scene.obj(filepath = path_to_file)

# Loop thorough all the objects in the scene and set shadows to false
#for object in scene.objects:
#    object.cycles_visibility.shadow = False
#    object.cycles_visibility.glossy = False
#    object.cycles_visibility.diffuse = False
#    object.cycles_visibility.transmission = False
#    object.cycles_visibility.scatter = False
#    object.cycles_visibility.camera = True
#    object.cycles.is_shadow_catcher = False

object = bpy.data.objects["wall_6_35"]
scene = bpy.context.scene 

#counting the number of objects and surfaces in the scene
object_count = 1
surfaces_count = 1 
print('Object count= %02d' %object_count) 
print('Name of the object is : ', object)
object_count += 1
mesh = object.data
mesh_owners = {}
print("Number of surfaces of this object= %d" % len(mesh.polygons))
print("--" *10)
for face in mesh.polygons: # iterate over face
    slot = object.material_slots[face.material_index]
    mat = slot.material
    print ('Surface count = %06d' %surfaces_count) 
    surfaces_count += 1
    print('Surface colour is : ',mat.diffuse_color)
    print("Number of vertices = %02d"  % len(mesh.vertices))
    for vert in mesh.vertices:
            print( 'v %f %f %f\n' % (vert.co.x, vert.co.y, vert.co.z) )
            #break
    print("--" *10)
print("."*80)  
