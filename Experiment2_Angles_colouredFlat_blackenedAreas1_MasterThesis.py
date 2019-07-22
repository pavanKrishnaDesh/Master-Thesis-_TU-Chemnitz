import bpy
import os
import math
import numpy as np
import bpy_extras
import mathutils
from math import radians
from mathutils import Matrix
from mathutils import Vector
from lxml import etree


object =bpy.context.object
scene = bpy.context.scene
pi = 3.141
scale = 0.04
scene.render.resolution_x= 1024
scene.render.resolution_y= 1024
scene.render.resolution_percentage = 100
scene.render.pixel_aspect_x = 1.00
scene.render.pixel_aspect_y = 1.00
resolution_x_in_px = scene.render.resolution_x
resolution_y_in_px = scene.render.resolution_y
pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
f_in_px =resolution_x_in_px/pi
scene.cycles.min_bounces = 0
scene.cycles.max_bounces = 4
scene.cycles.glossy_bounces = 0
#scene.render.tile_x = 1024/4
#scene.render.tile_y = 1024/4
scene.render.use_simplify = True
scene.cycles.device = 'GPU'
scene.render.engine = 'CYCLES'
scene.world.light_settings.use_ambient_occlusion = True
#scene.cycles.use_transparent_shadows = False
#Cycles.Visibility.Settings.shadow = False

#max_angle_deg = 70 # 70
#stepsize = 10

max_angle_deg = 11 # 70
stepsize = 1 #10 

caZRotInDeg = -120
cbZRotInDeg = -30

outdirPrefix = '/home/pavd/Desktop/mt_omnistereo/MasterThesis_Angles_related/Coloured_angles_blackenedAreas1_Experiment2/CalibrationFiles_Experiment2_ColoredSurfaces/'

def baselinecalc(p0,p1,p2):
    baseline0 = (p1 - p0).length
    baseline1 = (p2 - p1).length
    baseline2 = (p0 - p1).length
    

    print("="*60) 
    print('The distance of camera 1 to 2 is = ' , baseline0)
    print('The distance of camera 2 to 3 is = ' , baseline1)
    print('The distance of camera 3 to 1 is = ' , baseline2)
    print("="*60) 
    
def angle_between(p0,p1,p2):
    #print('The first vector is : ', p0)
    #print('The second vector is : ', p1)
    #print('The third vector is : ', p2)
    
    v0 = p1 - p0
    v1 = p2 - p1
    v2 = p0 - p2
    
    Cam1_cx = p1[0] - p0[0]
    Cam1_cy = p1[1] - p0[1]
    Cam1_cz = p1[2] - p0[2]
    
    Cam2_cx = repr(p2[0] - p0[0])
    Cam2_cy = repr(p2[1] - p0[1])
    Cam2_cz = repr(p2[2] - p0[2])
    
    
    print("="*100) 
    print('The location of camera 0 is = ' ,p0)
    print('The location of camera 1 is = ' ,p1)
    print('The location of camera 2 is = ' ,p2)
    print("="*100) 
    
    print("="*100)
    print('The Extrinsic data of camera C1 is')
    print(' C1 >> cx = ', Cam1_cx)
    print(' C1 >> cy = ', Cam1_cy)
    print(' C1 >> cz = ', Cam1_cz)
    print("="*100)
    
    print("="*100)
    print('The Extrinsic data of camera C2 is ')
    print(' C2 >> cx = ', Cam2_cx)
    print(' C2 >> cy = ', Cam2_cy)
    print(' C2 >> cz = ', Cam2_cz)
    print("="*100) 
    
    angle1 = np.arccos(np.dot(v0,-v2) / (np.linalg.norm(v0) * np.linalg.norm(-v2)))
    angle2 = np.arccos(np.dot(-v0,v1) / (np.linalg.norm(-v0) * np.linalg.norm(v1)))
    angle3 = np.arccos(np.dot(-v1,v2) / (np.linalg.norm(-v1) * np.linalg.norm(v2)))
    
    print("="*100) 
    print(' Camera 0 is at an angle of : ', np.degrees(angle1), 'degrees')
    print(' Camera 1 is at an angle of : ', np.degrees(angle2), 'degrees')
    print(' Camera 2 is at an angle of : ', np.degrees(angle3), 'degrees')    
    print("="*100) 
    
def calibration_matrix_K(camd):

    # Parameters of intrinsic calibration matrix K
    #alpha_x = f_in_mm/(resolution_x_in_px*scale/ sensor_width_in_mm)
    #alpha_y = f_in_mm/(resolution_y_in_px*scale*pixel_aspect_ratio/ sensor_height_in_mm)
    alpha_x = f_in_px
    alpha_y = f_in_px
    c_x = resolution_x_in_px / 2
    c_y = resolution_y_in_px / 2
    skew = 0 # only use rectangular pixels
    
    Kalibration = Matrix(((alpha_x, skew, c_x), (0, alpha_y, c_y),(0 , 0, 1 )))
    K = np.matrix(Kalibration)
    return K
    
#Extrinsic parameters, R and T
def _3x4_RT_matrix(cam):
    # bcam stands for blender camera
    R_bcam2cv = Matrix(
        ((1, 0,  0),
         (0, -1, 0),
         (0, 0, -1)))

    # Using matrix_world to account for all constraints
    location, rotation = cam.matrix_world.decompose()[0:2]
    R_world2bcam = rotation.to_matrix().transposed()

    # Converting camera location to translation vector used in coordinate changes
    T_world2bcam = -1*R_world2bcam*cam.location

    # Build the coordinate transform matrix from world to computer vision camera
    R_world2cv = R_bcam2cv*R_world2bcam
    T_world2cv = R_bcam2cv*T_world2bcam

    # put into 3x4 matrix
    RandT = Matrix((
        R_world2cv[0][:] + (T_world2cv[0],),
        R_world2cv[1][:] + (T_world2cv[1],),
        R_world2cv[2][:] + (T_world2cv[2],)
         ))
    RT =np.matrix(RandT)
    return RT

def _3x4_P_matrix(cam):
    K = calibration_matrix_K(cam.data)
    RT = _3x4_RT_matrix(cam)
    P = K*RT
    return P

def rotation_matrix(axis, theta, vector):
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    point = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])
    T = np.dot(point, vector)
    return T
    
def deg2rad(alpha):
    return alpha*np.pi/180.0

def baseline2positions(baseline, center):
    #a = 1/2*math.sqrt((baseline*baseline)/(1-math.cos(2/3 * math.pi))) #length of the shift vector 
    a = baseline / np.sqrt(2 * (1 - np.cos(2/3 * np.pi)))
    alphaRadians = deg2rad(120)
    c, s = np.cos(alphaRadians), np.sin(alphaRadians)
    rotArray = [[c,-s,0], [s,c,0], [0,0,1]];
    rotMatrix = np.matrix(rotArray)
    t0 = np.matrix([[0],[1],[0]])
    t1 = rotMatrix*t0
    t2 = np.transpose(rotMatrix)*t0
    print("="*100)
    print("Translation Vector t0 : \n", t0)
    print("="*100)
    print("Translation Vector t1 : \n", t1)
    print("="*100)
    print("Translation Vector t2 : \n", t2)
    print("="*100)
    center = np.matrix([[center[0]],[center[1]], [center[2]]])
    print("The center coordinates of the flat are : \n ", center)  
    print("="*100)
    c0 = center + a * t0
    c1 = center + a * t1
    c2 = center + a * t2
    return (c0, c1, c2)    #returns the camera positions


#def calibPhysCamToDist(phys_cam_data, filename):
def physicalCam(path,p0,p1,p2, psi1, theta1, phi1):
    # creating XML 
    root = etree.Element('Camera')
    ###########################################
    child = etree.Element('Body')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ############################################
    child = etree.Element('Lens')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ###########################################
    child = etree.Element('CalibrationData')
    ##########################################
    child1 = etree.Element('IntrinsicData')
    child1.attrib['model'] = 'Equiangular Kannala 9'
    child.append(child1)
    ##############################################
    child2 = etree.Element('ImageHeight')
    child2.text = '1680'
    child1.append(child2)
    #############################################
    child3 = etree.Element('ImageWidth')
    child3.text = '1680'
    child1.append(child3)
    #############################################
    child4 = etree.Element('ImageOrigin')
    child4.text = '1'
    child1.append(child4)
    ############################################
    child5 = etree.Element('fc1')
    child5.text = '534.7606088'
    child1.append(child5)
    #############################################
    child6 = etree.Element('fc2')
    child6.text = '534.7606088'
    child1.append(child6)
    #############################################
    child7 = etree.Element('alpha')
    child7.text = '0'
    child1.append(child7)
    #############################################
    child8 = etree.Element('cc1')
    child8.text = '840.50'
    child1.append(child8)
    #############################################
    child9 = etree.Element('cc2')
    child9.text = '840.50'
    child1.append(child9)
    #############################################
    child17 = etree.Element('k1')
    child17.text = '1'
    child1.append(child17)
    ############################################
    child18 = etree.Element('k2')
    child18.text = '0'
    child1.append(child18)
    ###########################################
    child19 = etree.Element('k3')
    child19.text = '0'
    child1.append(child19)
    #############################################
    child20 = etree.Element('k4')
    child20.text = '0'
    child1.append(child20)
    ##############################################
    child21 = etree.Element('k5')
    child21.text = '0'
    child1.append(child21)
    ############################################
    child10 = etree.Element('ExtrinsicData')
    child.append(child10)
    child10.attrib['translationUnit'] = 'Meter'
    ############################################
    child11 = etree.Element('cx')
    child10.append(child11)
    child11.text = str(p0)
    ############################################
    child12 = etree.Element('cy')
    child10.append(child12)
    child12.text = str(p1)
    ############################################
    child13 = etree.Element('cz')
    child10.append(child13)
    child13.text = str(p2)
    ############################################
    child14 = etree.Element('rx')
    child10.append(child14)
    child14.text = str(psi1)
    ############################################
    child15 = etree.Element('ry')
    child10.append(child15)
    child15.text = str(theta1)
    ############################################
    child16 = etree.Element('rz')
    child10.append(child16)
    child16.text = str(phi1)
    ############################################
    root.append(child)
    s = etree.tostring(root, pretty_print=True)
    #print(s)
    with open(path, "wb") as f:
        f.write(s)

#def calibVirtCamToDist(phys_cam_data, rot_z, filename):
def virtCamC0a(path,i,p0):
 # creating XML 
    root = etree.Element('Camera')
    ###########################################
    child = etree.Element('Body')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ############################################
    child = etree.Element('Lens')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ###########################################
    child = etree.Element('CalibrationData')
    ##########################################
    child1 = etree.Element('IntrinsicData')
    child1.attrib['model'] = 'EpipolarEquidistance No Distortion'
    child.append(child1)
    ##############################################
    child2 = etree.Element('ImageHeight')
    child2.text = '1024'
    child1.append(child2)
    #############################################
    child3 = etree.Element('ImageWidth')
    child3.text = '1024'
    child1.append(child3)
    #############################################
    child4 = etree.Element('ImageOrigin')
    child4.text = '1'
    child1.append(child4)
    ############################################
    child5 = etree.Element('fc1')
    child5.text = '325.9493234522016678056389'
    child1.append(child5)
    #############################################
    child6 = etree.Element('fc2')
    child6.text = '325.9493234522016678056389'
    child1.append(child6)
    #############################################
    child7 = etree.Element('alpha')
    child7.text = '0'
    child1.append(child7)
    #############################################
    child8 = etree.Element('cc1')
    child8.text = '512.5'
    child1.append(child8)
    #############################################
    child9 = etree.Element('cc2')
    child9.text = '512.5'
    child1.append(child9)
    #############################################
    child17 = etree.Element('k1')
    child17.text = '1'
    child1.append(child17)
    ############################################
    child18 = etree.Element('k2')
    child18.text = '0'
    child1.append(child18)
    ###########################################
    child19 = etree.Element('k3')
    child19.text = '0'
    child1.append(child19)
    #############################################
    child20 = etree.Element('k4')
    child20.text = '0'
    child1.append(child20)
    ##############################################
    child21 = etree.Element('k5')
    child21.text = '0'
    child1.append(child21)
    ############################################
    child10 = etree.Element('ExtrinsicData')
    child.append(child10)
    child10.attrib['translationUnit'] = 'Meter'
    ############################################
    child11 = etree.Element('cx')
    child10.append(child11)
    child11.text = str(p0[0])
    ############################################
    child12 = etree.Element('cy')
    child10.append(child12)
    child12.text = str(p0[1])
    ############################################
    child13 = etree.Element('cz')
    child10.append(child13)
    child13.text = str(p0[2])
    ############################################
    child14 = etree.Element('rx')
    child10.append(child14)
    child14.text = str(np.deg2rad(i))
    ############################################
    child15 = etree.Element('ry')
    child10.append(child15)
    child15.text = '0'
    ############################################
    child16 = etree.Element('rz')
    child10.append(child16)
    child16.text = str(np.deg2rad(caZRotInDeg))
    ############################################
    root.append(child)
    s = etree.tostring(root, pretty_print=True)
    #print(s)
    with open(path,"wb") as f:
        f.write(s)

def virtCamC1a(path,p1,i):
  # creating XML 
    root = etree.Element('Camera')
    ###########################################
    child = etree.Element('Body')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ############################################
    child = etree.Element('Lens')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ###########################################
    child = etree.Element('CalibrationData')
    ##########################################
    child1 = etree.Element('IntrinsicData')
    child1.attrib['model'] = 'EpipolarEquidistance No Distortion'
    child.append(child1)
    ##############################################
    child2 = etree.Element('ImageHeight')
    child2.text = '1024'
    child1.append(child2)
    #############################################
    child3 = etree.Element('ImageWidth')
    child3.text = '1024'
    child1.append(child3)
    #############################################
    child4 = etree.Element('ImageOrigin')
    child4.text = '1'
    child1.append(child4)
    ############################################
    child5 = etree.Element('fc1')
    child5.text = '325.9493234522016678056389'
    child1.append(child5)
    #############################################
    child6 = etree.Element('fc2')
    child6.text = '325.9493234522016678056389'
    child1.append(child6)
    #############################################
    child7 = etree.Element('alpha')
    child7.text = '0'
    child1.append(child7)
    #############################################
    child8 = etree.Element('cc1')
    child8.text = '512.5'
    child1.append(child8)
    #############################################
    child9 = etree.Element('cc2')
    child9.text = '512.5'
    child1.append(child9)
    #############################################
    child17 = etree.Element('k1')
    child17.text = '1'
    child1.append(child17)
    ############################################
    child18 = etree.Element('k2')
    child18.text = '0'
    child1.append(child18)
    ###########################################
    child19 = etree.Element('k3')
    child19.text = '0'
    child1.append(child19)
    #############################################
    child20 = etree.Element('k4')
    child20.text = '0'
    child1.append(child20)
    ##############################################
    child21 = etree.Element('k5')
    child21.text = '0'
    child1.append(child21)
    ############################################
    child10 = etree.Element('ExtrinsicData')
    child.append(child10)
    child10.attrib['translationUnit'] = 'Meter'
    ############################################
    child11 = etree.Element('cx')
    child10.append(child11)
    child11.text = str(p1[0])
    ############################################
    child12 = etree.Element('cy')
    child10.append(child12)
    child12.text = str(p1[1])
    ############################################
    child13 = etree.Element('cz')
    child10.append(child13)
    child13.text = str(p1[2])
    ############################################
    child14 = etree.Element('rx')
    child10.append(child14)
    child14.text = str(np.deg2rad(i))
    ############################################
    child15 = etree.Element('ry')
    child10.append(child15)
    child15.text = '0'
    ############################################
    child16 = etree.Element('rz')
    child10.append(child16)
    child16.text = str(np.deg2rad(caZRotInDeg)) 
    ############################################
    root.append(child)
    s = etree.tostring(root, pretty_print=True)
    #print(s)
    with open(path, "wb") as f:
        f.write(s)


def virtCamC0b(path,i,p0):
 # creating XML 
    root = etree.Element('Camera')
    ###########################################
    child = etree.Element('Body')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ############################################
    child = etree.Element('Lens')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ###########################################
    child = etree.Element('CalibrationData')
    ##########################################
    child1 = etree.Element('IntrinsicData')
    child1.attrib['model'] = 'EpipolarEquidistance No Distortion'
    child.append(child1)
    ##############################################
    child2 = etree.Element('ImageHeight')
    child2.text = '1024'
    child1.append(child2)
    #############################################
    child3 = etree.Element('ImageWidth')
    child3.text = '1024'
    child1.append(child3)
    #############################################
    child4 = etree.Element('ImageOrigin')
    child4.text = '1'
    child1.append(child4)
    ############################################
    child5 = etree.Element('fc1')
    child5.text = '325.9493234522016678056389'
    child1.append(child5)
    #############################################
    child6 = etree.Element('fc2')
    child6.text = '325.9493234522016678056389'
    child1.append(child6)
    #############################################
    child7 = etree.Element('alpha')
    child7.text = '0'
    child1.append(child7)
    #############################################
    child8 = etree.Element('cc1')
    child8.text = '512.5'
    child1.append(child8)
    #############################################
    child9 = etree.Element('cc2')
    child9.text = '512.5'
    child1.append(child9)
    #############################################
    child17 = etree.Element('k1')
    child17.text = '1'
    child1.append(child17)
    ############################################
    child18 = etree.Element('k2')
    child18.text = '0'
    child1.append(child18)
    ###########################################
    child19 = etree.Element('k3')
    child19.text = '0'
    child1.append(child19)
    #############################################
    child20 = etree.Element('k4')
    child20.text = '0'
    child1.append(child20)
    ##############################################
    child21 = etree.Element('k5')
    child21.text = '0'
    child1.append(child21)
    ############################################
    child10 = etree.Element('ExtrinsicData')
    child.append(child10)
    child10.attrib['translationUnit'] = 'Meter'
    ############################################
    child11 = etree.Element('cx')
    child10.append(child11)
    child11.text = str(p0[0])
    ############################################
    child12 = etree.Element('cy')
    child10.append(child12)
    child12.text = str(p0[1])
    ############################################
    child13 = etree.Element('cz')
    child10.append(child13)
    child13.text = str(p0[2])
    ############################################
    child14 = etree.Element('rx')
    child10.append(child14)
    child14.text = str(np.deg2rad(i))
    ############################################
    child15 = etree.Element('ry')
    child10.append(child15)
    child15.text = '0'
    ############################################
    child16 = etree.Element('rz')
    child10.append(child16)
    child16.text = str(np.deg2rad(cbZRotInDeg))
    ############################################
    root.append(child)
    s = etree.tostring(root, pretty_print=True)
    #print(s)
    with open(path, "wb") as f:
        f.write(s)     
     
def virtCamC2b(path, p2,i):
 # creating XML 
    root = etree.Element('Camera')
    ###########################################
    child = etree.Element('Body')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ############################################
    child = etree.Element('Lens')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ###########################################
    child = etree.Element('CalibrationData')
    ##########################################
    child1 = etree.Element('IntrinsicData')
    child1.attrib['model'] = 'EpipolarEquidistance No Distortion'
    child.append(child1)
    ##############################################
    child2 = etree.Element('ImageHeight')
    child2.text = '1024'
    child1.append(child2)
    #############################################
    child3 = etree.Element('ImageWidth')
    child3.text = '1024'
    child1.append(child3)
    #############################################
    child4 = etree.Element('ImageOrigin')
    child4.text = '1'
    child1.append(child4)
    ############################################
    child5 = etree.Element('fc1')
    child5.text = '325.9493234522016678056389'
    child1.append(child5)
    #############################################
    child6 = etree.Element('fc2')
    child6.text = '325.9493234522016678056389'
    child1.append(child6)
    #############################################
    child7 = etree.Element('alpha')
    child7.text = '0'
    child1.append(child7)
    #############################################
    child8 = etree.Element('cc1')
    child8.text = '512.5'
    child1.append(child8)
    #############################################
    child9 = etree.Element('cc2')
    child9.text = '512.5'
    child1.append(child9)
    #############################################
    child17 = etree.Element('k1')
    child17.text = '1'
    child1.append(child17)
    ############################################
    child18 = etree.Element('k2')
    child18.text = '0'
    child1.append(child18)
    ###########################################
    child19 = etree.Element('k3')
    child19.text = '0'
    child1.append(child19)
    #############################################
    child20 = etree.Element('k4')
    child20.text = '0'
    child1.append(child20)
    ##############################################
    child21 = etree.Element('k5')
    child21.text = '0'
    child1.append(child21)
    ############################################
    child10 = etree.Element('ExtrinsicData')
    child.append(child10)
    child10.attrib['translationUnit'] = 'Meter'
    ############################################
    child11 = etree.Element('cx')
    child10.append(child11)
    child11.text = str(p2[0])
    ############################################
    child12 = etree.Element('cy')
    child10.append(child12)
    child12.text = str(p2[1])
    ############################################
    child13 = etree.Element('cz')
    child10.append(child13)
    child13.text = str(p2[2])
    ############################################
    child14 = etree.Element('rx')
    child10.append(child14)
    child14.text = str(np.deg2rad(i))
    ############################################
    child15 = etree.Element('ry')
    child10.append(child15)
    child15.text = '0'
    ############################################
    child16 = etree.Element('rz')
    child10.append(child16)
    child16.text = str(np.deg2rad(cbZRotInDeg))
    ############################################
    root.append(child)
    s = etree.tostring(root, pretty_print=True)
    #print(s)
    with open(path, "wb") as f:
        f.write(s)

def NoDistCam(path):
    # creating XML 
    root = etree.Element('Camera')
    ###########################################
    child = etree.Element('Body')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ############################################
    child = etree.Element('Lens')
    child.append(etree.Element('Manufacturer'))
    child.append(etree.Element('Type'))
    child.append(etree.Element('SerialNumber'))
    root.append(child)
    ###########################################
    child = etree.Element('CalibrationData')
    ##########################################
    child1 = etree.Element('IntrinsicData')
    child1.attrib['model'] = 'Equiangular No Distortion'
    child.append(child1)
    ##############################################
    child2 = etree.Element('ImageHeight')
    child2.text = '1024'
    child1.append(child2)
    #############################################
    child3 = etree.Element('ImageWidth')
    child3.text = '1024'
    child1.append(child3)
    #############################################
    child4 = etree.Element('ImageOrigin')
    child4.text = '1'
    child1.append(child4)
    ############################################
    child5 = etree.Element('fc1')
    child5.text = '325.9493234522016678056389'
    child1.append(child5)
    #############################################
    child6 = etree.Element('fc2')
    child6.text = '325.9493234522016678056389'
    child1.append(child6)
    #############################################
    child7 = etree.Element('alpha')
    child7.text = '0'
    child1.append(child7)
    #############################################
    child8 = etree.Element('cc1')
    child8.text = '512.5'
    child1.append(child8)
    #############################################
    child9 = etree.Element('cc2')
    child9.text = '512.5'
    child1.append(child9)
    #############################################
    child10 = etree.Element('ExtrinsicData')
    child.append(child10)
    child10.attrib['translationUnit'] = 'Meter'
    ############################################
    child11 = etree.Element('cx')
    child10.append(child11)
    child11.text = '0'
    ############################################
    child12 = etree.Element('cy')
    child10.append(child12)
    child12.text = '0'
    ############################################
    child13 = etree.Element('cz')
    child10.append(child13)
    child13.text = '0'
    ############################################
    child14 = etree.Element('rx')
    child10.append(child14)
    child14.text = '0'
    ############################################
    child15 = etree.Element('ry')
    child10.append(child15)
    child15.text = '0'
    ############################################
    child16 = etree.Element('rz')
    child10.append(child16)
    child16.text = '0'
    ############################################
    root.append(child)
    s = etree.tostring(root, pretty_print=True)
    #print(s)
    with open(path, "wb") as f:
        f.write(s)


def rotMat2angles(mat_rotated):
    if (mat_rotated[2,0] != +1) and (mat_rotated[2,0] != -1):
        theta1 = -np.arcsin(mat_rotated[2,0])
        theta2 = np.pi -theta1
        
        psi1 = np.arctan2((mat_rotated[2,1]/np.cos(theta1)),(mat_rotated[2,2]/np.cos(theta1)))
        psi2 = np.arctan2((mat_rotated[2,1]/np.cos(theta2)),(mat_rotated[2,2]/np.cos(theta2)))
        
        phi1 = np.arctan2((mat_rotated[1,0]/np.cos(theta1)),(mat_rotated[0,0]/np.cos(theta1)))
        phi2 = np.arctan2((mat_rotated[1,0]/np.cos(theta2)),(mat_rotated[0,0]/np.cos(theta2)))

    else:
        phi1 = 0
        phi2 = 0
        if(mat_rotated[2,0] == -1):
            theta1 = np.pi/2
            psi1 = phi1 + np.arctan2(mat_rotated[0,1],mat_rotated[0,2])
        else:
            theta2 = -np.pi/2
            psi2 = -phi2 + np.arctan2(-mat_rotated[0,1],-mat_rotated[0,2])
    
    print("The values of psi (in radians) are",psi1,"and",psi2)
    print("The values of theta (in radians) are",theta1,"and" ,theta2)
    print("The values of phi (in radians) are",phi1,"and",phi2)
    print("."*100)
    
    print("The values of psi (in degrees) are",np.rad2deg(psi1),"and",np.rad2deg(psi2))
    print("The values of theta (in degrees) are",np.rad2deg(theta1),"and",np.rad2deg(theta2))
    print("The values of phi (in degrees) are",np.rad2deg(phi1),"and",np.rad2deg(phi2))
    print("="*100)  
            
    return psi1, theta1, phi1, psi2, theta2, phi2

def angles_Cam0(cam_object0):

    scene = bpy.data.scenes["Scene"]
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.quality = 100
    bpy.context.scene.render.image_settings.color_mode = 'RGB'
    print("*"*100) 
    print("."*100) 
    print("The next block is of the Camera 0")
    print("*"*100) 
    print("."*100) 
    
    for i in range(0, max_angle_deg, stepsize):
        mat_rot_Cam0 = mathutils.Matrix.Rotation(radians(i), 4, 'X')
        mat_rotated = np.matrix(mat_rot_Cam0)
        #cam_object0.matrix_world = mat_rot 
        bpy.data.objects["Camera 0"].rotation_euler[0] = radians(i) 
        
        p0 = cam_object0.location
        p1 = cam_object1.location
        p2 = cam_object2.location
        
        print('The location of camera 0 is = ' ,p0)
        print('The location of camera 1 is = ' ,p1)
        print('The location of camera 2 is = ' ,p2)
        
        print("="*100) 
        print("Rotation Matrix_Cam0_%02d_degree_rotation is : \n" %i ,mat_rotated)
        print("."*100) 
        
        #print("value at 2,2 is", mat_rotated[2,2]) to check for the individual element values
        #psi1, theta1, phi1, psi2, theta2, phi2 = rotMat2angles(mat_rotated)
    
        scene.objects.active = cam_object0
        scene.camera = cam_object0
        bpy.context.object.data.type = 'PANO'
        bpy.context.object.data.cycles.panorama_type = 'FISHEYE_EQUIDISTANT'
        #scene.render.filepath = '/home/pavd/Desktop/mt_omnistereo/MasterThesis_Angles_related/Coloured_angles_blackenedAreas1_Experiment2/C0/camop0_%02d_degree_rotation.png' %i
        #bpy.ops.render.render(write_still=True)
            
        outDir = outdirPrefix + '%02d_degree_rotation/' %i
        if (os.path.isdir(outDir) == False):
            os.makedirs(outDir)
        
        NoDistCam(outDir +'C0_noDist_%02d_degree_rotation.xml' %i)
        physicalCam(outDir + 'C0_%02d_degree_rotation.xml' %i, p0[0], p0[1], p0[2], np.deg2rad(i), 0, 0)
        virtCamC0a(outDir + 'C0a_%02d_degree_rotation.xml' %i,i,p0)
        virtCamC0b(outDir + 'C0b_%02d_degree_rotation.xml' %i,i,p0)
        
        bpy.context.scene.update()   

def angles_Cam1(cam_object1):

    scene = bpy.data.scenes["Scene"]
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.quality = 100
    bpy.context.scene.render.image_settings.color_mode = 'RGB'
    
    p0 = cam_object0.location
    p1 = cam_object1.location
    p2 = cam_object2.location
    
    print('The location of camera 0 is = ' ,p0)
    print('The location of camera 1 is = ' ,p1)
    print('The location of camera 2 is = ' ,p2)
    
    print("*"*100) 
    print("."*100) 
    print("The next block is of the Camera 1")
    print("*"*100) 
    print("."*100) 
    for i in range(0, max_angle_deg, stepsize):
       
        #cam_object0.matrix_world = mat_rot 
        #bpy.data.objects["Camera 1"].rotation_euler[2] = radians(90) 
        #mat_rot_psi = mathutils.Matrix.Rotation(radians(90), 4, 'Z')
        
        bpy.data.objects["Camera 1"].rotation_euler[2] = radians(120)
        mat_rot_theta = mathutils.Matrix.Rotation(radians(120), 4, 'Z')
        
        bpy.data.objects["Camera 1"].rotation_euler[0] = radians(i) 
        mat_rot_phi = mathutils.Matrix.Rotation(radians(i), 4, 'X')
        
        mat_rot = mat_rot_phi*mat_rot_theta
        mat_rotated = np.matrix(mat_rot)
        
        print("="*100) 
        print("Rotation Matrix_Cam1_%02d_degree_rotation is : \n" %i ,mat_rotated)
        print("."*100) 
        
        #psi1, theta1, phi1, psi2, theta2, phi2 = rotMat2angles(mat_rotated)
        
        scene.objects.active = cam_object1
        scene.camera = cam_object1
        bpy.context.object.data.type = 'PANO'
        bpy.context.object.data.cycles.panorama_type = 'FISHEYE_EQUIDISTANT'
        #scene.render.filepath = '/home/pavd/Desktop/mt_omnistereo/MasterThesis_Angles_related/Coloured_angles_blackenedAreas1_Experiment2/C1/camop1_%02d_degree_rotation.png' %i
        #bpy.ops.render.render(write_still=True)
        
        outDir = outdirPrefix + '%02d_degree_rotation/' %i
        if (os.path.isdir(outDir) == False):
            os.makedirs(outDir)

        physicalCam(outDir + 'C1_%02d_degree_rotation.xml' %i,p1[0], p1[1], p1[2], np.deg2rad(i), 0, np.deg2rad(120))
        virtCamC1a(outDir +'C1a_%02d_degree_rotation.xml' %i, p1,i)
        bpy.context.scene.update()   
        
        
def angles_Cam2(cam_object2):

    scene = bpy.data.scenes["Scene"]
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.quality = 100
    bpy.context.scene.render.image_settings.color_mode = 'RGB'
    
    p0 = cam_object0.location
    p1 = cam_object1.location
    p2 = cam_object2.location
    
    print('The location of camera 0 is = ' ,p0)
    print('The location of camera 1 is = ' ,p1)
    print('The location of camera 2 is = ' ,p2)
    
    print("*"*100) 
    print("."*100) 
    print("The next block is of the Camera 2")
    print("*"*100) 
    print("."*100) 
    
    for i in range(0, max_angle_deg, stepsize):
        #mat_rot = mathutils.Matrix.Rotation(10, 4, 'X')
        #cam_object0.matrix_world = mat_rot 
        
        bpy.data.objects["Camera 2"].rotation_euler[2] = radians(-120)
        mat_rot_theta = mathutils.Matrix.Rotation(radians(-120), 4, 'Z')
        
        bpy.data.objects["Camera 2"].rotation_euler[0] = radians(i) 
        mat_rot_phi = mathutils.Matrix.Rotation(radians(i), 4, 'X')
        
        mat_rot = mat_rot_phi*mat_rot_theta
        mat_rotated = np.matrix(mat_rot)  
        
        print("="*100) 
        print("Rotation Matrix_Cam2_%02d_degree_rotation is : \n" %i ,mat_rotated)
        print("."*100) 
        
        # psi1, theta1, phi1, psi2, theta2, phi2 = rotMat2angles(mat_rotated)
        
        scene.objects.active = cam_object2
        scene.camera = cam_object2
        bpy.context.object.data.type = 'PANO'
        bpy.context.object.data.cycles.panorama_type = 'FISHEYE_EQUIDISTANT'
        #scene.render.filepath = '/home/pavd/Desktop/mt_omnistereo/MasterThesis_Angles_related/Coloured_angles_blackenedAreas1_Experiment2/C2/camop2_%02d_degree_rotation.png' %i
        #bpy.ops.render.render(write_still=True)
 
        outDir = outdirPrefix + '%02d_degree_rotation/' %i
        if (os.path.isdir(outDir) == False):
            os.makedirs(outDir)
        
        physicalCam(outDir + 'C2_%02d_degree_rotation.xml' %i ,p2[0], p2[1], p2[2], np.deg2rad(i), 0, np.deg2rad(-120))
        virtCamC2b(outDir +'C2b_%02d_degree_rotation.xml' %i,p2,i)
    
        bpy.context.scene.update()      

def experiment_1(cam_object0, cam_object1, cam_object2):
    center = [11,-13,10]   #coordinates of the midpoint of the scaled flat
    baseline = 15
    base_scaled = baseline * scale
    
    
    # calculating positions of all cameras acording to the baseline
    # setting the camera object locations
    (c0,c1,c2) =  baseline2positions(base_scaled,center)
    cam_object0.location = tuple(c0.flatten().tolist()[0])
    cam_object1.location = tuple(c1.flatten().tolist()[0])
    cam_object2.location = tuple(c2.flatten().tolist()[0])
    p0 = cam_object0.location
    p1 = cam_object1.location
    p2 = cam_object2.location
    
    #angle_between(p0,p1,p2)
      

# from cm to blender unit 1 cm = 0.04 BU , 1 BU = 25cm
#bpy.ops.script.python_file_run(filepath="/usr/share/blender/2.79/scripts/presets/units_length/centimeters.py")
#scene.unit_settings.system = 'METRIC'
#scene.unit_settings.scale_length = 1
            

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
path_to_obj_dir = '/home/pavd/Desktop/mt_omnistereo/MasterThesis_Angles_related/Coloured_angles_blackenedAreas1_Experiment2/Flat_blackened_coloured1/'
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
    
# create camera data
cam_data0 = bpy.data.cameras.new("Camera 0")
cam_data1 = bpy.data.cameras.new("Camera 1")
cam_data2 = bpy.data.cameras.new("Camera 2")
    
#create new objects with camera datablock       
cam_object0 = bpy.data.objects.new(name = "Camera 0", object_data = cam_data0)
cam_object1 = bpy.data.objects.new(name = "Camera 1", object_data = cam_data1)
cam_object2 = bpy.data.objects.new(name = "Camera 2", object_data = cam_data2)

#link cameras to the scene
scene.objects.link(cam_object0)
scene.objects.link(cam_object1)
scene.objects.link(cam_object2)

#place cameras on specified location
#cam_object1.location = (245 * scale, -340 * scale, 235* scale)
#cam_object0.location = (300 * scale, -255 * scale, 235 * scale)
#cam_object2.location = (345 * scale, -345 * scale, 235 * scale)

#p0 = cam_object0.location
#p1 = cam_object1.location
#p2 = cam_object2.location

#cliiping the far plane
#bpy.data.cameras[cam_object1.name].clip_end = 10000
#bpy.data.cameras[cam_object2.name].clip_end = 10000
#bpy.data.cameras[cam_object3.name].clip_end = 10000

#Creating new lamp datablock
lamp_data1 = bpy.data.lamps.new(name = "lamp1", type = "POINT")
lamp_data2 = bpy.data.lamps.new(name = "lamp2", type = "POINT")

#Creating new lamp datablock (using SUN as light source for better illumination)
#lamp_data1 = bpy.data.lamps.new(name = "lamp1", type = "SUN")
#lamp_data2 = bpy.data.lamps.new(name = "lamp2", type = "SUN")

bpy.data.lamps['lamp1'].cycles.cast_shadow = False
bpy.data.lamps['lamp2'].cycles.cast_shadow = False

#setting the pont source energy to maximum
#lamp_data1.energy = 1000000
#lamp_data2.energy = 1000000

#lamp_data1.falloff_type = 'INVERSE_SQUARE'
#lamp_data2.falloff_type = 'INVERSE_SQUARE'
#lamp_data1.distance = 1000
#lamp_data2.distance = 1000

#creating new objects with lamp datablocks
lamp_object1 = bpy.data.objects.new(name = "lamp1", object_data = lamp_data1)
lamp_object2 = bpy.data.objects.new(name = "lamp2", object_data = lamp_data2)

#linking lamp objects to the scene
bpy.context.scene.objects.link(lamp_object1)
bpy.context.scene.objects.link(lamp_object2)

#placing lamps at specified location
lamp_object1.location = (295 * scale, -200 * scale, 232 * scale)
lamp_object2.location = (295 * scale, -400 * scale, 232 * scale)

#making both th lamps active
lamp_object1.select = True
scene.objects.active = lamp_object1
lamp_object2.select = True
scene.objects.active = lamp_object2

#print('The location of camera 0 is = ' ,p0)
#print('The location of camera 1 is = ' ,p1)
#print('The location of camera 2 is = ' ,p2)

#print(p0.length)
#print(p1.length)
#print(p2.length)

experiment_1(cam_object0, cam_object1, cam_object2)


p0 = cam_object0.location
p1 = cam_object1.location
p2 = cam_object2.location

Cam1_cx = p1[0] - p0[0]
Cam1_cy = p1[1] - p0[1]
Cam1_cz = p1[2] - p0[2]

Cam2_cx = p2[0] - p0[0]
Cam2_cy = p2[1] - p0[1]
Cam2_cz = p2[2] - p0[2]

#calibration_matrix_K(cam_data0)
#calibration_matrix_K(cam_data1)
#calibration_matrix_K(cam_data2)
#_3x4_RT_matrix(cam_object0)
#_3x4_RT_matrix(cam_object1)
#_3x4_RT_matrix(cam_object2)
#baselinecalc(p0,p1,p2)
#angle_between(p0,p1,p2) 
angles_Cam0(cam_object0)
angles_Cam1(cam_object1)
angles_Cam2(cam_object2)

#K0 = calibration_matrix_K(cam_data0)
#K1 = calibration_matrix_K(cam_data1)
#K2 = calibration_matrix_K(cam_data2)

#RT0 = _3x4_RT_matrix(cam_object0)
#RT1 = _3x4_RT_matrix(cam_object1)
#RT2 = _3x4_RT_matrix(cam_object2)

#P0 =  _3x4_P_matrix(cam_object0)
#P1 =  _3x4_P_matrix(cam_object1)
#P2 =  _3x4_P_matrix(cam_object2)


#vector0 = p0
#vector1 = p1
#vector2 = p2
#value1 = np.deg2rad(30)
#theta = value1
#axis = [0,0,1]

#RotMatrix0 = rotation_matrix(axis, theta, vector0)
#RotMatrix1 = rotation_matrix(axis, theta, vector1)
#RotMatrix2 = rotation_matrix(axis, theta, vector2)


#if __name__ == "__main__":
#    print("="*100)
#    #K_0 = calibration_matrix_K(K0)
#    print('Calibration matrix - Cam0 : ', K0)
#    #K_1 = calibration_matrix_K(K1)
#    print('Calibration matrix - Cam1 :', K1)
#    K_2 = calibration_matrix_K(K2)
#    print('Calibration matrix - Cam2 :', K2)
#    print("="*100) 
    
#    print("="*100)
#    print('RT0 - Cam0 :', RT0)
#    print('RT1 - Cam1 :', RT1)
#    print('RT2 - Cam2 :', RT2)
#    print("="*100)
    
#    print("="*100)
#    print('P0 - Cam0 :', P0)
#    print('P1 - Cam1 :', P1)
#    print('P2 - Cam2 :', P2)
#    print("="*100)
    
#    print("="*100)
#    print('Rot - Cam0 :', RotMatrix0)
#    print('Rot - Cam1 :', RotMatrix1)
#    print('Rot - Cam2 :', RotMatrix2)
#    print("="*100)
    
print("finished")
