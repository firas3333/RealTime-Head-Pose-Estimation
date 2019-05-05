import cv2
import numpy as np
import ProvingClass
from glob import glob
import codecs , json
import math
import ProvingClass
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
DataPath="/home/odroid/Firass Project IO Data/Calibration Output Data"

file_directory4 = DataPath+"/Camera1IntrisicsMAT.json"
json_data4 = open(file_directory4).read()
data4= json.loads(json_data4)
#Loading Cmaera matrix from the calibration
K1= data4['camera1']['camera matrix']
file_directory5 = DataPath+"/Camera2IntrisicsMAT.json"
json_data5 = open(file_directory5).read()
data5 = json.loads(json_data5)
#Loading Cmaera matrix from the calibration
K2= data5['camera2']['camera matrix']
#~ Proj13x4=np.delete(T1,3,0)
#~ Proj23x4=np.delete(T2,3,0)
#~ lop=np.dot(K1,Proj13x4)
#~ lop2=np.dot(K2,Proj23x4)




img_counter = 1
corner=[]
corners= np.asarray(corner)
corner2=[]
corners2= np.asarray(corner2)
master= ProvingClass.Calibration(1,(9,6),32)
calib= ProvingClass.Calibration(1,(9,6),32)
calib2= ProvingClass.Calibration(2,(9,6),32)
RTs = DataPath+"/Transform12.json"
json_data = open(RTs).read()
data = json.loads(json_data)
TransformationMatrix= data['Trans12']['Transform matrix']


MATRICES=[]
Images=[]
lala=[]
NumOCams=3
Plotpoint=[]
#getting all the calibrated cameras matrix stored in array 
#~ ProvingClass.GetCamsMatrix(master,NumOCams)
#Getting all the transformation matrices (pre proccess extrinsics calib) stored in array Tc=[]
#~ ProvingClass.GetTansMats(master,NumOCams)
#start the		
a=0
print 180/math.pi

cam = cv2.VideoCapture(0)
cam2 = cv2.VideoCapture(1)
while True: 
	ret, frame = cam.read()
	ret2, frame2 = cam2.read()
	cv2.imshow("test", frame)
	cv2.imshow("test2", frame2)
	img = frame
	img2= frame2
	ProvingClass.SetPatternPoints(calib)
	ProvingClass.SetPatternPoints(calib2)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img2= cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
	calib.h, calib.w = img.shape[:2]
	calib2.h, calib2.w = img2.shape[:2]
	Images.extend([img,img2])
	found, calib.corners = cv2.findChessboardCorners(img, calib.pattern_size)
	found2, calib2.corners = cv2.findChessboardCorners(img2, calib2.pattern_size)
	if a>=1:
		#~ ProvingClass.surf(Images)
		print 'okey done'
	elif found: # andrey 1
				print 'ok andrey'  
				calib.term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 10, 0.1 )
				cv2.cornerSubPix(img, calib.corners, (5, 5), (-1, -1), calib.term)
				calib.img_points.append(calib.corners.reshape(-1, 2).astype(np.float32))
				calib.obj_points.append(calib.pattern_points.astype(np.float32))
				calib2.term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 10, 0.1 )
				cv2.cornerSubPix(img2, calib2.corners, (5, 5), (-1, -1), calib2.term)
				calib2.img_points.append(calib2.corners.reshape(-1, 2).astype(np.float32))
				calib2.obj_points.append(calib2.pattern_points.astype(np.float32))
				ProvingClass.SolvepnpTmatFrame1(calib,K1)
				ProvingClass.SolvepnpTmatFrame1(calib2,K2)
				othertrans=np.dot(TransformationMatrix,calib.a)
				Rotationmatrix=np.array([[othertrans[0][0][0],othertrans[0][0][1],othertrans[0][0][2]],
				[othertrans[0][1][0],othertrans[0][1][1],othertrans[0][1][2]], 
				[othertrans[0][2][0],othertrans[0][2][1],othertrans[0][2][2]]])
				Translatevec=np.array([[othertrans[0][0][3]],[othertrans[0][1][3]],[othertrans[0][2][3]]])
				otherCamerasPositions=-np.matrix(np.asarray(Rotationmatrix)).T*np.matrix(np.asarray(Translatevec))
				x4=(math.atan2(othertrans[0][2][1],othertrans[0][2][2]))*(180/math.pi)
				y4=(math.atan2(-(othertrans[0][2][0]),math.sqrt(othertrans[0][2][1]*othertrans[0][2][1]+othertrans[0][2][2]*othertrans[0][2][2])))*(180/math.pi)
				z4=(math.atan2(othertrans[0][1][0],othertrans[0][0][0]))*(180/math.pi)
				print 'cam2pos'
				print otherCamerasPositions
				print 'cam2 rotation vec from Transformation'
				print (x4,y4,z4)
				print 'xxxxxxxxxxxxxxxx'
				OT3x4=np.delete(othertrans,3,1)
				OT34=(np.reshape(OT3x4,(3,4)))
				rightproj=np.dot(K2,OT34)
				print 'cam1 Trans'
				print calib.herllo
				print '---'
				print calib.lop
				print 'cam2 Trans'
				print calib2.herllo
				print '---'
				print 'cam2 Trans from Trasformation'
				print OT34
				



				for i in range(0,len(calib.img_points[0]-1)):
				
					tmp_rightproj = np.asarray(rightproj)
					tmp_calib2 = np.asarray(calib2.lop)
					#T
					tmp_rightproj[0][3] = tmp_calib2[0][3]
					tmp_rightproj[1][3] = tmp_calib2[1][3]
					tmp_rightproj[2][3] = tmp_calib2[2][3]
					#R
					#~ tmp_rightproj[0][0] = tmp_calib2[0][0]
					#~ tmp_rightproj[0][1] = tmp_calib2[0][1]
					#~ tmp_rightproj[0][2] = tmp_calib2[0][2]
					#~ tmp_rightproj[1][0] = tmp_calib2[1][0]
					#~ tmp_rightproj[1][1] = tmp_calib2[1][1]
					#~ tmp_rightproj[1][2] = tmp_calib2[1][2]
					#~ tmp_rightproj[2][0] = tmp_calib2[2][0]
					#~ tmp_rightproj[2][1] = tmp_calib2[2][1]
					#~ tmp_rightproj[2][2] = tmp_calib2[2][2]
					#~ bob= cv2.triangulatePoints(np.asarray(calib.lop,dtype=np.float32),np.asarray(calib2.lop,dtype=np.float32),np.asarray(calib.img_points[0][i],dtype=np.float32),np.asarray(calib2.img_points[0][i],dtype=np.float32))	
					bob= cv2.triangulatePoints(np.asarray(calib.lop,dtype=np.float32),np.asarray(rightproj,dtype=np.float32),np.asarray(calib.img_points[0][i],dtype=np.float32),np.asarray(calib2.img_points[0][i],dtype=np.float32))	
					#~ bob=cv2.triangulatePoints(np.asarray(calib.lop,dtype=np.float32),tmp_rightproj,np.asarray(calib.img_points[0][i],dtype=np.float32),np.asarray(calib2.img_points[0][i],dtype=np.float32))
					bob=bob/bob[3]
					tria3=np.delete(bob,3,0)
					lala.extend([tria3])
				print '3d chess points'	
				print lala
				for i in range(0,len(lala)):
					Plotpoint.extend([lala[i]])
				rvec , tvec, inliers = cv2.solvePnPRansac(np.asarray(lala,dtype=np.float32), np.asarray(calib.img_points[0],dtype=np.float32) , np.asarray(K1,dtype=np.float64),None)
				rotationMat= cv2.Rodrigues(rvec)[0]
				cameraPos= -np.matrix(rotationMat).T*np.matrix(tvec)
				rvec2 , tvec2 ,inliers2= cv2.solvePnPRansac(np.asarray(lala,dtype=np.float32), np.asarray(calib2.img_points[0],dtype=np.float32) , np.asarray(K2,dtype=np.float64),None)
				aha= np.array([[rotationMat[0][0],rotationMat[0][1],rotationMat[0][2], tvec[0]],
					[rotationMat[1][0],rotationMat[1][1],rotationMat[1][2], tvec[1]],
					[rotationMat[2][0],rotationMat[2][1],rotationMat[2][2], tvec[2]],
					[0,0,0,1]],dtype=np.float32)
				rotationMat2= cv2.Rodrigues(rvec2)[0]
				cameraPos2= -np.matrix(rotationMat2).T*np.matrix(tvec2)
				aha2= np.array([[rotationMat2[0][0],rotationMat2[0][1],rotationMat2[0][2], tvec2[0]],
					[rotationMat2[1][0],rotationMat2[1][1],rotationMat2[1][2], tvec2[1]],
					[rotationMat2[2][0],rotationMat2[2][1],rotationMat2[2][2], tvec2[2]],
					[0,0,0,1]],dtype=np.float32)
				x4=(math.atan2(rotationMat[2][1],rotationMat[2][2]))*(180/math.pi)
				y4=(math.atan2(-(rotationMat[2][0]),math.sqrt(rotationMat[2][1]*rotationMat[2][1]+rotationMat[2][2]*rotationMat[2][2])))*(180/math.pi)
				z4=(math.atan2(rotationMat[1][0],rotationMat[0][0]))*(180/math.pi)
				print'cam2 proj from Transformation'
				print tmp_rightproj
				print 'jojojo'
				print tmp_calib2
				print 'cam1 pos solvepnp after tria'
				print cameraPos
				print 'Trans cam1 solvepnp after tria'
				print aha
				print 'cam1 pos solvepnp after tria'
				print cameraPos2
				print 'Trans cam1 solvepnp after tria'
				print aha2
				fig = plt.figure()
				ax = fig.add_subplot(111, projection='3d')
				ax.set_xlabel('x')
				ax.set_ylabel('y')
				ax.set_zlabel('z')
				ax.set_xlim(-300,300)
				ax.set_ylim(-300,300)
				ax.set_zlim(-300,300)
				for i in range(0,len(lala)):
					plt.plot([Plotpoint[i][0][0]], [Plotpoint[i][1][0]], [Plotpoint[i][2][0]], 'or')
				fig = plt.figure()
				ax = fig.add_subplot(111)
				ax.set_xlabel('x')
				ax.set_ylabel('y')
				for i in range(0,len(calib.obj_points[0])):
					plt.plot([calib.img_points[0][i][0]], [calib.img_points[0][i][1]], 'or')
				fig = plt.figure()
				ax = fig.add_subplot(111)
				ax.set_xlabel('x')
				ax.set_ylabel('y')
				for i in range(0,len(calib2.img_points[0])):
					plt.plot([calib2.img_points[0][i][0]], [calib2.img_points[0][i][1]], 'or')
				plt.show()
				break		
				print 'ok andrey'         
				cv2.drawChessboardCorners(img, calib.pattern_size, calib.corners, found)
				a+=1
	elif found2: #camer 0
				print 'ok0'  
				calib.term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
				cv2.cornerSubPix(img, calib.corners, (5, 5), (-1, -1), calib.term)
				calib.img_points.append(calib.corners.reshape(-1, 2).astype(np.float32))
				calib.obj_points.append(calib.pattern_points.astype(np.float32))
				calib2.term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
				cv2.cornerSubPix(img2, calib2.corners, (5, 5), (-1, -1), calib2.term)
				calib2.img_points.append(calib2.corners.reshape(-1, 2).astype(np.float32))
				calib2.obj_points.append(calib2.pattern_points.astype(np.float32))
				ProvingClass.SolvepnpTmatFrame1(calib,K1)
				ProvingClass.SolvepnpTmatFrame1(calib2,K2)
				print calib.obj_points[0]
				for i in range(0,len(calib.img_points[0])):
					bob=cv2.triangulatePoints(np.asarray(calib.lop,dtype=np.float32),np.asarray(calib2.lop,dtype=np.float32),np.asarray(calib.img_points[0][i],dtype=np.float32),np.asarray(calib2.img_points[0][i],dtype=np.float32))
					bob=bob/bob[3]
					tria3=np.delete(bob,3,0)
					lala.extend([tria3])
				for i in range(0,len(lala)):
					Plotpoint.append(lala[i])
			
				fig = plt.figure()
				ax = fig.add_subplot(111, projection='3d')
				ax.set_xlabel('x')
				ax.set_ylabel('y')
				ax.set_zlabel('z')

				ax.set_xlim3d(-300,300)
				ax.set_ylim3d(-300,300)
				ax.set_zlim3d(-300,300)
				for i in range(0,len(lala)):
					plt.plot([Plotpoint[i][0][0]], [Plotpoint[i][1][0]], [Plotpoint[i][2][0]], 'or')
				plt.show()
				a+=1
	else:
		
		print 'no chessboard in first frame YOURE DOING IT WRONG'
		a=0
	
	
	
	cv2.waitKey(1)
cam.release()

cv2.destroyAllWindows()
