import bpy
import bmesh
import os
import numpy as np
import math

object = bpy.context.object
scene = bpy.context.scene 
tau = 0.27999

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
for object in scene.objects:
    object.cycles_visibility.shadow = False
    object.cycles_visibility.glossy = False
    object.cycles_visibility.diffuse = False
    object.cycles_visibility.transmission = False
    object.cycles_visibility.scatter = False
    object.cycles_visibility.camera = True
    object.cycles.is_shadow_catcher = False

#counting the number of objects and surfaces in the scene
yellow_wall_5_29                            = (0.9888, 1.0000, 0.0000)
blue_wall_6_35                              = (0.0000, 0.0000, 1.0000)
cadetblue_142                               = (0.3725, 0.6196, 0.6275)
#deeppink_Cube_1_1_4_123                     = (0.8000, 0.0624, 0.6400)
darkolivegreen_Cube_Cube_Material_118       = (0.0727, 0.1176, 0.0227)
orangered_97                                = (1.0000, 0.2706, 0.0000)
goldenrod_103                               = (0.8549, 0.6471, 0.1225)
green_wall_8_47                             = (0.0000, 0.8000, 0.0044)
cyan_wall_1_4                               = (0.0000, 1.0000, 1.0000)
saddlebrown_wall_2_10                       = (0.5451, 0.2706, 0.0745)
red_diningtable_205                         = (0.8000, 0.0000, 0.0000)

obj1 = yellow_wall_5_29
obj2 = blue_wall_6_35
obj3 = cadetblue_142
#obj4 = deeppink_Cube_1_1_4_123
obj5 = darkolivegreen_Cube_Cube_Material_118
obj6 = orangered_97
obj7 = goldenrod_103
obj8 = green_wall_8_47
obj9 = cyan_wall_1_4
obj10 = saddlebrown_wall_2_10
obj11 = red_diningtable_205

colors_list = [obj1, obj2, obj3, obj5, obj6, obj7, obj8, obj9, obj10, obj11]
    
#colors_list = [0.9888, 1.0000, 0.0000, 0.3725, 0.6196, 0.6275,0.8000, 0.0624, 0.6400,0.0727, 
                #0.1176, 0.0227,0.2706, 0.8549, 0.6471, 0.1225, 0.0044,0.5451, 0.2706, 0.0745, 0.8000]
                
#equations = [
#[1,2,3,4],
#[4,4,2,4]
#]               
                             
object_count = 1
surfaces_count = 1 
for object in scene.objects:
    print("."*80) 
    print('Object count= %02d' %object_count) 
    print('Name of the object is : ', object)
    object_count += 1
    mesh = object.data
    mesh_owners = {}
    print("Number of surfaces of this object= %d" % len(mesh.polygons))
    print("--" *10)
    for face in mesh.polygons: # iterate over surfaces
        slot = object.material_slots[face.material_index]
        mat = slot.material
        print ("Surface count = %06d" %surfaces_count) 
        surfaces_count += 1
        print("Surface colour is : ", mat.diffuse_color)
        print("Number of vertices = %02d"  % len(mesh.vertices))
        for vert in mesh.vertices:
            print( 'v %f %f %f\n' % (vert.co.x, vert.co.y, vert.co.z) )
            break
        #print('type:',  mat.diffuse_color.type)
        #print('Memory Location of surface = ',face)
        #print("--" *10)
        for c in colors_list:
            #print("c0 is", float(c[0]))
            #print("diff0 is", mat.diffuse_color[0])
            (c1, c2, c3) = c
            (d1, d2, d3) = mat.diffuse_color
            dist = math.sqrt((c1-d1)**2 + (c2-d2)**2 + (c3-d3)**2)
            if dist < 1e-6:
                print("Now, we found a non-black surface")
            #print("color found")
        print("--" *10)
        
        
        #for i in range(len(colors_list)):
            #c = colors_list[i]
            # is mat.diffuse_color approximately out color c?
            #dist = math.sqrt((c[0] - mat.diffuse_color[0])**2 + ......)
            #if dist < tau:
            #    print("Point found")
            #else
            #    print("Point ignored")
            #equation = equations[i] # parameters in a python list\
            ## TODO: calculate 3D distance between point 
            #print("Color is : ", c)
    print("."*80)  


#cl = np.array(colors_list)


