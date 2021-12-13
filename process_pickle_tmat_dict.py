import pickle
import math
import numpy as np
# from geometry import getRmatrix, rpy2rot
from geometry import unit
# from draw3d import BBox3D, drawBox
import cv2
from open3dtest import draw_registration_result, depth2pointcloud
import open3d as o3d
import copy

# tmat_experiment_gt
#tmat_M01_gt
with open('tmat_experiment_gt.p', 'rb') as f:
    dict_tmat_gt = pickle.load(f)

#tmat_experiment_pred
#tmat_M01_pred
#tmat_pred.p
with open('tmat_S05_gt.p', 'rb') as f:
    dict_tmat_pred = pickle.load(f)

tmat_0 = dict_tmat_gt[5]
tran0 = tmat_0[0:3,3]
rot0 = tmat_0[0:3,0:3]



print('0000 Tmat:\n', tmat_0)
print('0000 Translation:\n', tmat_0[0:3,3])
print('0000 Rotation:\n', tmat_0[0:3,0:3])

tmat_1 = dict_tmat_pred[7]
tran1 = tmat_1[0:3,3]
# tran1[1] = 1*tran1[1]
# tran1[2] = 1*tran1[2]
rot1 = tmat_1[0:3,0:3]
# rot1[1] = 1*rot1[1]
# rot1[2] = 1*rot1[2]

print('\n0001 Tmat:\n', tmat_1)
print('0001 Translation:\n', tran1)
print('0001 Rotation:\n', rot1)


example007fromme = np.array([[ 0.15163723, -0.04049065,  0.98760653,  0.06322413],
                             [ 0.07752873,  0.99656959,  0.02895436, -0.00419794],
                             [-0.98539101,  0.07217733,  0.15425624,  0.46515322],
                             [ 0.        ,  0.        ,  0.        ,  1.        ]])

example007fromchin = np.array([[-0.02774127,  0.01593274, -0.99948815,  0.26139251],
                               [-0.5646254 ,  0.82484396,  0.0288202 ,  0.11791958],
                               [ 0.82488095,  0.56513591, -0.01388619, -0.63399957],
                               [ 0.        ,  0.        ,  0.        ,  1.        ]])

# array([[-0.02774127, -0.01593274,  0.99948815,  0.26139251],
#        [-0.5646254 , -0.82484396, -0.0288202 ,  0.11791958],
#        [ 0.82488095, -0.56513591,  0.01388619, -0.63399957],
#        [ 0.        ,  0.        ,  0.        ,  1.        ]])

example = np.array([[0.004891778179209212, -0.011032558541465136, 0.9999271739273488, 0.3357646907712071],
                    [0.0681100697179112, 0.9976207125529266, 0.010673906899306756, -0.009069887394181428],
                    [-0.9976658202571211 , 0.06805289514415291, 0.005631567564222464, 0.6584533800826952],
                    [0.0, 0.0, 0.0, 1.0]])

examplerot = example[0:3,0:3]
exampletran = example[0:3,3]

simplerot = np.array([  [0.7603140, -0.6495557,  0.0000000],
                      [0.6495557,  0.7603140,  0.0000000],
                      [0.0000000,  0.0000000,  1.0000000] ])

startingrot = np.array([[  0.0007963,  0.0000000,  0.9999997],
                        [0.0000000,  1.0000000,  0.0000000],
                        [-0.9999997,  0.0000000,  0.0007963 ]])

tiltedrot = np.array([[ -0.4993102,  0.0000000,  0.8664233],
                      [0.0000000,  1.0000000,  0.0000000],
                      [-0.8664233,  0.0000000, -0.4993102 ]])

def rotationMatrixToEulerAngles(R) :

    # assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def creffopen():
    #this part is fixed. This is arm wrt the camera.  
    #we get instructions from camera, hence need to convert to global arm instruction
    x = np.array([0,-1,0]) *-1
    # y = np.array([0,0,-1])*-1
    # z = np.array([1,0,0]) 
    
    y = np.array([-0.5,0,-0.8660254037]) *-1    
    z = np.array([0.8660254037,0,-0.5]) *1

   
    
    # x = np.array([0,0,1])
    # y = np.array([-1,0,0]) 
    # z = np.array([0,-1,0])    
    
    
    # x = np.array([0,-1,0])
    # y = np.array([0.866,0,-0.5]) 
    # z = np.array([0.5,0.866,0])
    
    # x = np.array([0,-0.5,0.866])
    # y = np.array([-1,0,0]) 
    # z = np.array([0.5,-0.866,0])
    
    eye = np.eye(3)
    
    refmat = np.array([[np.dot(x,eye[i]) for i in range(3)],
                        [np.dot(y,eye[i]) for i in range(3)],
                        [np.dot(z,eye[i]) for i in range(3)]])
    # print(x)
    print("referencematrix",refmat)
    return refmat

def crefftrans():
    #this part is fixed. This is arm wrt the camera.  
    #we get instructions from camera, hence need to convert to global arm instruction
    x = np.array([0,-1,0]) 
    y = np.array([-0.5,0,-0.8660254037]) 
    z = np.array([0.8660254037,0,-0.5]) #np.array([1,0,0]) #i know that its the reverse 30 degree, hence i take the last 3 and flip only the x to -ve


    # x = np.array([0,-0.5,0.866])
    # y = np.array([-1,0,0]) 
    # z = np.array([0.5,-0.866,0])
    
    eye = np.eye(3)
    
    refmat = np.array([[np.dot(x,eye[i]) for i in range(3)],
                        [np.dot(y,eye[i]) for i in range(3)],
                        [np.dot(z,eye[i]) for i in range(3)]])
    # print(refmat)
    return refmat

def crefftest():
    #this part is fixed. This is arm wrt the camera.  
    #we get instructions from camera, hence need to convert to global arm instruction
    # x = np.array([0,-1,0]) 
    # y = np.array([-0.5,0,-0.8660254037]) 
    # z = np.array([0.8660254037,0,-0.5]) #np.array([1,0,0]) #i know that its the reverse 30 degree, hence i take the last 3 and flip only the x to -ve

    x = np.array([0,0.5,-0.866])
    y = np.array([-1,0,0]) 
    z = np.array([0,0.866,0.5])

    # x = np.array([0,-0.5,0.866])
    # y = np.array([-1,0,0]) 
    # z = np.array([0,-0.866,-0.5])
    
    eye = np.eye(3)
    
    refmat = np.array([[np.dot(x,eye[i]) for i in range(3)],
                        [np.dot(y,eye[i]) for i in range(3)],
                        [np.dot(z,eye[i]) for i in range(3)]])
    # print(refmat)
    return refmat

def creffrot():
    #this part is fixed. This is arm wrt the camera.  
    #we get instructions from camera, hence need to convert to global arm instruction
    x = np.array([0,-1,0])  *-1
    y = np.array([-0.5,0,-0.8660254037]) *-1
    z = np.array([0.8660254037,0,-0.5]) #np.array([1,0,0]) #i know that its the reverse 30 degree, hence i take the last 3 and flip only the x to -ve


    # x = np.array([0,-0.5,0.866])
    # y = np.array([-1,0,0]) 
    # z = np.array([0.5,-0.866,0])
    
    eye = np.eye(3)
    
    refmat = np.array([[np.dot(x,eye[i]) for i in range(3)],
                        [np.dot(y,eye[i]) for i in range(3)],
                        [np.dot(z,eye[i]) for i in range(3)]])
    return refmat

def getRmatrix(roll,pitch,yaw):
    
    yawMatrix = np.matrix([[math.cos(yaw), -math.sin(yaw), 0],
                           [math.sin(yaw), math.cos(yaw), 0],
                           [0, 0, 1]])

    pitchMatrix = np.matrix([[math.cos(pitch), 0, math.sin(pitch)],
                             [0, 1, 0],
                             [-math.sin(pitch), 0, math.cos(pitch)]])

    rollMatrix = np.matrix([[1, 0, 0],
                            [0, math.cos(roll), -math.sin(roll)],
                            [0, math.sin(roll), math.cos(roll)]])
    
    R = yawMatrix * pitchMatrix * rollMatrix
    # R = rollMatrix * pitchMatrix *  yawMatrix
    
    return R

def rotateonspot( mat , axis, theta):
    identity = np.eye(3)
    
    
    # zaxis = np.array([0,0,1]) #using z axis

    
    rad = (3.142*theta)/180
    
    W = np.array([[0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]])

    rod = identity + np.sin(rad)*W + (1-np.cos(rad)) * np.matmul(W,W)
    newv = np.matmul(rod,mat)
    return newv
    

drefframeT=creffopen()
drefframeTranslation=crefftrans()

print("camera translation: ",exampletran)
robottran = np.matmul(exampletran,drefframeTranslation)

newt = np.matmul(drefframeT,rot1)
# newt = np.matmul(examplerot,drefframeT)
result = np.matmul(newt,tiltedrot)
result = np.matmul(getRmatrix(0,0,1.57),result)
# result = np.matmul(startingrot,newt)


print("\nstarting vector: ",rotationMatrixToEulerAngles(tiltedrot))
print("rotation in camera axis: ",rotationMatrixToEulerAngles(rot1) *180/3.142)

print("rotation in robot: ",rotationMatrixToEulerAngles(newt)*180/3.142)

print("result in robot: ",rotationMatrixToEulerAngles(result)*180/3.142)

# a= rotationMatrixToEulerAngles(result)
# R = getRmatrix(a[0],a[1],a[2])

theta = math.acos(((result[0, 0] + result[1, 1] + result[2, 2]) - 1) / 2)
multi = 1 / (2 * math.sin(theta))


rx = multi * (result[2, 1] - result[1, 2]) * theta
ry = multi * (result[0, 2] - result[2, 0]) * theta
rz = multi * (result[1, 0] - result[0, 1]) * theta
outputrotationvector = [rx,ry,rz]
print("Quaternion: ", outputrotationvector)

#for 14, [ 0.07347906 0.13298431  -0.69089588] camera
# for 14, [ 0.66482553 -0.07347906 -0.23028015] robo

# translation = np.array([-0.11828865, -0.34707721 , 0.84397872])
translation = tran1

drefframeTest=crefftest()

print("\nexample translation: ",translation)
newtt = np.matmul(translation,drefframeTest.transpose())
print("example transation in robot",newtt)
# print(rotationMatrixToEulerAngles(newt))

# newttest = np.matmul(translation,drefframeTest.transpose())
newttest = np.matmul(drefframeTest,translation)
print(newttest)

print("Try example rotation ")
rottest = np.matmul(drefframeTest,examplerot)
print(drefframeTest)
print(rottest)
print("rotation in robot: ",rotationMatrixToEulerAngles(rottest)*180/3.142)


pcdimport = "S05_GL3cut2_open_axisadj3scaled.ply"

source = o3d.io.read_point_cloud(pcdimport)    #combine3 and combine3cone testcombine0246cone combine0246357cone
                                            #S05_GL3cut2_open_axisadj3scaled
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])                                            
path = 'temp/'
numbers = 0 #edge cases are 0,11,12,21,22,32 can try all these
target = depth2pointcloud(numbers,path)
# draw_registration_result(source, target, tmat_1)
# tmat12 = tmat_1
# tmat12[1] = -1*tmat12[1]
# tmat12[2] = -1*tmat12[2]
source_1 = copy.deepcopy(source)
source_2 = copy.deepcopy(source)
source_3 = copy.deepcopy(source)
source.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
target.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

# pcd.rotate(R, center=(0, 0, 0))
# pcd.translate((0,0,450)) 

# drefframeTest[0] = -drefframeTest[0] 
# drefframeTest[1] = -drefframeTest[1]  

# drefframeTest[2] = -drefframeTest[2] 
# a = np.linalg.inv(drefframeTest)

test = np.array([1,0,0])
test = np.eye(3)

# b = rotateonspot(test,np.array([1,0,0]),-30)
# b = rotateonspot(b,np.array([0,1,0]),-90)
# b = rotateonspot(b,np.array([1,0,0]),90)

b = rotateonspot(test,np.array([1,0,0]),60)
b = rotateonspot(b,np.array([0,-0.5,-0.866]),90)
# b = rotateonspot(b,np.array([0,1,0]),90)
# b = rotateonspot(b,np.array([0,0,1]),-90)
source_1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
source_1.rotate(b,center=(0, 0, 0))
source_2.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

tmatrot = tmat_1[0:3,0:3]
c = np.matmul(b,tmatrot)

# source_2.rotate(tmat_1[0:3,0:3],center=(0, 0, 0))
# source_2.rotate(b,center=(0, 0, 0))


source_2.rotate(c,center=(0, 0, 0))

t = np.matmul(b,translation)
source_2.translate((t))


print(t)
zeroTmat = np.array([[-0.020199014497708126, -0.0014813494302840884, 0.9997948816718394, -0.30897824332215235],
                      [0.05405290158995731, 0.9985347620918521, 0.0025715217100360493, -0.00010441897143767922],
                      [-0.9983337536330439, 0.05409375655345232, -0.020089346900982005, 0.47269374250636054],
                      [0.0, 0.0, 0.0, 1.0]])

source_3.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# source_3.rotate([[1, 0, 0 ], [0, 1, 0], [0, 0, 1]],center=(0, 0, 0))
top = np.eye(3)
# top = rotateonspot(top,np.array([1,0,0]),180)
# top = rotateonspot(top,np.array([0,1,0]),180)
# source_3.rotate(top,center=(0, 0, 0))

# abc = np.array([[-0.63381824, -0.19138197, -0.74943137, -0.18303574],[-0.65503334,  0.64808429,  0.38848177,  0.0153746 ],[ 0.41134629,  0.73712936, -0.53612921, -0.45156396],[0,0,0,1]])
#example007fromchin


abc = np.array([[0.75496627, -0.06390151, -0.65264273, -0.18166028],[-0.34171636,  0.81109933, -0.47470812,  0.03365453 ],[ 0.55969265,  0.58140732,  0.59051644, -0.38855642],[0,0,0,1]])

source_3.transform(abc)


o3d.visualization.draw_geometries([source_3,target,axis],
                                      top = 100,
                                      left = 0,
                                      zoom= 0.45,
                                      front=[0,0,-1],
                                      lookat=[0,0,0],
                                      up=[0.0,-1.0,0.0])

d = np.matmul(c,tiltedrot)

print("camera rotation last: ",rotationMatrixToEulerAngles(tmatrot)*180/3.142)
print("robot rotation last: ",rotationMatrixToEulerAngles(c)*180/3.142)
print("rotation in robot last: ",rotationMatrixToEulerAngles(d)*180/3.142)

# print("result in robot last: ",rotationMatrixToEulerAngles(result)*180/3.142)

theta = math.acos(((d[0, 0] + d[1, 1] + d[2, 2]) - 1) / 2)
multi = 1 / (2 * math.sin(theta))


rx = multi * (d[2, 1] - d[1, 2]) * theta
ry = multi * (d[0, 2] - d[2, 0]) * theta
rz = multi * (d[1, 0] - d[0, 1]) * theta
outputrotationvector = [rx,ry,rz]
print("Quaternion: ", outputrotationvector)

y = np.matmul(drefframeTest,translation)
drefrot = drefframeTest
drefrot[0] = 1*drefrot[0]
drefrot[1] = 1*drefrot[1]
yyrot = np.matmul(tmatrot,drefframeTest)
print(y)
print("camera rotation last: ",rotationMatrixToEulerAngles(yyrot)*180/3.142)

drefframeT=creffrot()
newt = np.matmul(drefframeT,rotateonspot(tmatrot,np.array([1,0,0]),180))
print("camera rotation last: ",rotationMatrixToEulerAngles(newt)*180/3.142)


