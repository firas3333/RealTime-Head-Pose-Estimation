
import numpy as np
import itertools
import cv2
import codecs,json
import cv2
import codecs , json
import numpy as np
import itertools
import os
from common import splitfn
import sys, getopt
from glob import glob
import ProvingClass
import matplotlib.pyplot as plt



# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
DataPath="/home/odroid/Firass Project IO Data/Calibration Output Data"
Extrinsicimage="/home/odroid/Firass Project IO Data/Images for extirinsic calib"  

#~ 
NumOCams=2
CP3=[]
imgpopo=[]
Tc=[]
CP4=[]
file_directory = DataPath+"/Transform12.json"
json_data = open(file_directory).read()
data = json.loads(json_data)
#Loading Cmaera matrix from the calibration
T12= data['Trans12']['Transform matrix']
a=[]
matrices=[]
for i in range(1,NumOCams+1):
	
#loading calibrated json files
		file_directory = DataPath+"/Camera"+ str(i)+"IntrisicsMAT.json"
		json_data = open(file_directory).read()
		data = json.loads(json_data)
#Loading Cmaera matrix from the calibration
		cameramatrice= data['camera'+ str(i)]['camera matrix']
#creating obj/img Points
		calib= ProvingClass.Calibration(i,(9,6),32)
		calib.img_mask = Extrinsicimage+'/cam'+str(i)+'.jpg'
		calib.img_names = glob(calib.img_mask)
		ProvingClass.SetPatternPoints(calib)
		ProvingClass.FindDrawChess(calib)
		ProvingClass.SolvepnpTmatFrame1(calib,cameramatrice)
		a.append(calib.a)
		CP3.extend([calib.cameraPos])
		imgpopo.extend([calib.img_points])
		matrices.append(cameramatrice)
for i in range(0,NumOCams):
	Camposvec4= np.vstack([CP3[i][0],[1]])
	CP4.extend([Camposvec4])
T121=np.reshape(T12,(4,4))
T12=np.dot((T121),(a[0])) 
T12=np.delete(T12,3,0)
print matrices[1]
project2=np.dot(matrices[1],T12)
Rotationmatrix=np.array([[T12[0][0],T12[0][1],T12[0][2]],
			[T12[1][0],T12[1][1],T12[1][2]], 
			[T12[2][0],T12[2][1],T12[2][2]]],dtype=np.float32)
Translatevec=np.array([[T12[0][3]],[T12[1][3]],[T12[2][3]]],dtype=np.float32)
othercameraPosi=-np.matrix(np.asarray(Rotationmatrix,dtype=np.float32)).T*np.matrix(np.asarray(Translatevec,dtype=np.float32))
print'------------- camera 1 position from solvepnp-------'
print CP3[0]
print'------------- camera 2 position from solvepnp-------'
print CP3[1]
print '-------- camera 2 position from the (-np.matrix(np.asarray(Rotationmatrix,dtype=np.float32)).T*np.matrix(np.asarray(Translatevec,dtype=np.float32)))---------'
print othercameraPosi
print 'trans 2'
print T12
print 'projection 2'
print project2
T12=np.dot(np.linalg.inv(T121),a[1]) 
T12=np.delete(T12,3,0)
project=np.dot(matrices[0],T12)
Rotationmatrix=np.array([[T12[0][0],T12[0][1],T12[0][2]],
			[T12[1][0],T12[1][1],T12[1][2]], 
			[T12[2][0],T12[2][1],T12[2][2]]],dtype=np.float32)
Translatevec=np.array([[T12[0][3]],[T12[1][3]],[T12[2][3]]],dtype=np.float32)
othercameraPosi=-np.matrix(np.asarray(Rotationmatrix,dtype=np.float32)).T*np.matrix(np.asarray(Translatevec,dtype=np.float32))
print '-------- camera 1 position from the (-np.matrix(np.asarray(Rotationmatrix,dtype=np.float32)).T*np.matrix(np.asarray(Translatevec,dtype=np.float32)))---------'
print othercameraPosi
print 'Trans 1'

print T12
print 'projection1'

print project
print'--------'
