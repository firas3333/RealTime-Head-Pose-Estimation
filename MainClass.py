import numpy as np
import cv2
import os
from common import splitfn
import sys, getopt
from glob import glob
import codecs , json
import operator as op
import matplotlib.pyplot as plt
from threading import Thread
from imutils.video import WebcamVideoStream
import math
import datetime


class WebcamVideoStream:
	def __init__(self, src):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.stream = cv2.VideoCapture(src)
		(self.grabbed, self.frame) = self.stream.read()

		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False

	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return

			# otherwise, read the next frame from the stream
			(self.grabbed, self.frame) = self.stream.read()
			

	def read(self):
		# return the frame most recently read
		return self.frame

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True
class Calibration:
	def __init__(self):
		#preprossess 
		self.pattern_points=[]
		self.pattern_size=[]
		self.square_size=[]
		# 3d point in real world space
 		self.obj_points= []
 		# 2d points in image plane.      
		self.img_points=[]
		self.TranformationMats=[]
		#THE loop
		self.calibdcams=[]
		self.TriaPoints3=[]
		self.cam1matchingarray=[]
		#for matching and triangulation
		self.listkp1=[]
		self.listkp2=[]
		self.matchingkp=[]
		self.gooddescs=[]
		#For solvePnpframe1
		self.oldcamerapos=[]
		self.oldTrans=[]
		#For solvePnpframe2
		self.otherCamerasPositions=[]
		self.othercameraProjMats=[]
		#timers
#for preprosses
def getcalibrationdetails(self,PathOfTextfile):
	path=PathOfTextfile
	params= open(path,"r")
	self.CalibDetails=[]
	for line in params :
		Cameraid=line[0]
		ChessBoardHeight=line[1:2]
		ChessBoradWidt=line[2:3]
		if Cameraid == '#':
			continue
		else :
			counter=0
			for p in line:
				counter+=1
				if p==',':
					Square_size=line[counter:]
					CBH=int(ChessBoardHeight)
					CBW=int(ChessBoradWidt)
					CameraId=int(Cameraid)
					Square_size=float(Square_size)
					pattern_size=(CBH,CBW)
					self.CalibDetails.extend([CameraId,pattern_size,Square_size])		
					self.pattern_size=pattern_size
					self.square_size=Square_size
#get images path and names each camera should has its own images file named Camera + the CameraId ( preffered 10-30 imgs)			
def GetImages(self,CameraId,path):
	img_mask= ''
	self.img_names={''}
	CameraId=CameraId
	img_mask = path + str(CameraId)+'/calib*.jpg'
	self.img_names = glob(img_mask)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
def SetPatternPoints(self,Ps):
	Ps=Ps
	self.pattern_points = np.zeros( (np.prod(self.CalibDetails[Ps+1]), 3), np.float32 )
	self.pattern_points[:,:2] = np.indices(self.CalibDetails[Ps+1]).T.reshape(-1, 2)
	self.pattern_points *= self.CalibDetails[Ps+2]	

#finds the chess corners and draws them on the actual image 	
def FindDrawChess(self,Ps):
	Ps=Ps
	for fn in self.img_names:
		print 'processing %s...' % fn,
		img = cv2.imread(fn, 0)
		self.h, self.w = img.shape[:2]
		found, self.corners = cv2.findChessboardCorners(img, self.CalibDetails[Ps+1])
		if found:
			term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
			cv2.cornerSubPix(img, self.corners, (5, 5), (-1, -1), term)
		if not found:
			print 'chessboard not found'    
			continue
		self.img_points.append(self.corners.reshape(-1, 2).astype(np.float32))
		self.obj_points.append(self.pattern_points.astype(np.float32))

		print 'ok'  
	#this part is only to draw it doesnt change any of the results (optional)         
		vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
		cv2.drawChessboardCorners(vis, self.CalibDetails[Ps+1], self.corners, found)
		cv2.imshow('img', vis)
		cv2.waitKey(500)	  				
		 				

#using solveon on the points w got from the chessboard pattern we can get the Transformation matrix from camera -> world we save it for later use
def Solvepnpforixtrinsic(self,cameramatrice):
		retval , rvec , tvec= cv2.solvePnP(np.asarray(self.obj_points,dtype=np.float32), np.asarray(self.img_points,dtype=np.float32) , np.asarray(cameramatrice,dtype=np.float64),None)
		rotationMat= cv2.Rodrigues(rvec)[0]
		a= np.array([[rotationMat[0][0],rotationMat[0][1],rotationMat[0][2], tvec[0]],
					[rotationMat[1][0],rotationMat[1][1],rotationMat[1][2], tvec[1]],
					[rotationMat[2][0],rotationMat[2][1],rotationMat[2][2], tvec[2]],
					[0,0,0,1]],dtype=np.float32)
		JsonTransf=a.tolist()
		self.TranformationMats.extend([JsonTransf])
		print len(self.TranformationMats)
		del self.obj_points[:]
		del self.img_points[:]
		
#saving the intrinsic matrixes into json files	
def SaveCalibratedCams(self,array,Path):
	CamerasArray=array
	j=0	
	for i in range(0, len(CamerasArray)/3):
		cam_List= {'camera'+str(CamerasArray[j]):{'camera matrix':  CamerasArray[j+1], 'distortion coef' :   CamerasArray[j+2]}}   
		jsonCamtMat = Path+str(CamerasArray[j])+"IntrisicsMAT.json"
		json.dump(cam_List, codecs.open(jsonCamtMat, 'w' , encoding = 'utf-8') , sort_keys = True , indent=4) 
		j=j+3	
		print 'data successfuly saved to :'	,jsonCamtMat
		

				
#saving The transformation of each camera to other cameras using T12=T2w * inv(T1w)
def savingTM_Rotate_trnaslate(calib,NumOcams,TranformationM):
	TranformationM=TranformationM
	NumOcams=NumOcams
	MATRICES=[]
	bpb=[]
	RTij=[]
	c=1
	for i in range(0,NumOcams-1):
		for j in range(c,NumOcams):
			inversetoMat= np.dot(TranformationM[j],np.linalg.inv(TranformationM[i]) )
			MATRICES.extend([inversetoMat])
		c+=1	
	for i in range (0,len(MATRICES)):
		TRansij=np.array([MATRICES[i]])

		liT=TRansij.tolist()
		bpb.extend([liT])
	
		j=0
		Rij = np.array([[MATRICES[i][0][0],MATRICES[i][0][1],MATRICES[i][0][2]],
					[MATRICES[i][1][0],MATRICES[i][1][1],MATRICES[i][1][2]], 
					[MATRICES[i][2][0],MATRICES[i][2][1],MATRICES[i][2][2]]],dtype=np.float32)
									
		Tij = np.array([MATRICES[i][j][3],MATRICES[i][j+1][3],MATRICES[i][j+2][3]],dtype=np.float32)
		ListR= Rij.tolist()
		ListT= Tij.tolist()
		RTij.extend([ListR,ListT])		
	b,e,c=0,0,1	
	for i in range(0,NumOcams-1):
		for j in range(c,NumOcams):
			RT_List= {'Trans'+str(i+1)+str(j+1):{'Transform matrix':  bpb[b]}}   
			RTs = "/home/odroid/Firass Project IO Data/Calibration Output Data/Transform"+str(i+1) + str(j+1) + ".json"
			json.dump(RT_List, codecs.open(RTs, 'w' , encoding = 'utf-8') , sort_keys = True , indent=4) 
			RT_List1= {'RT'+str(i+1)+str(j+1):{'R':  RTij[e], 'T' :   RTij[e+1]}}   
			RTs1 = "/home/odroid/Firass Project IO Data/Calibration Output Data/RT"+str(i+1)+str(j+1)+".json"
			json.dump(RT_List, codecs.open(RTs1, 'w' , encoding = 'utf-8') , sort_keys = True , indent=4) 
			e+=2 
			b+=1
			print 'data successfuly saved to :'	,RTs1
			print 'Transformation matrixes successfuly saved to :'	,RTs
		c+=1
				
#for the LOOP	----------------------------------------------------------------------------------------------------
#loading intrinsic matrix for all cams		 						
def GetCamsMatrix(self,NumOCams):
	for i in range(1,NumOCams+1):
		file_directory = "/home/odroid/Firass Project IO Data/Calibration Output Data/Camera"+ str(i)+"IntrisicsMAT.json"
		json_data = open(file_directory).read()
		data = json.loads(json_data)
		cameramatrice= data['camera'+ str(i)]['camera matrix']
		self.calibdcams.extend([cameramatrice])

#getting all the Transformation matrixs that are to the maincam
def GetTansMats(self,maincam,NumOCams):
	self.allcams=[]
	self.TransFromMainToOther=[]
	c=1
	for i in range(0,NumOCams-1):
		self.Tc=[]
		for j in range(c,NumOCams):  
			RTs = "/home/odroid/Firass Project IO Data/Calibration Output Data/Transform"+str(i+1)+str(j+1)+".json"
			json_data = open(RTs).read()
			data = json.loads(json_data)
			TransformationMatrix= data['Trans'+str(i+1)+str(j+1)]['Transform matrix']
			self.Tc.append([TransformationMatrix])
		self.allcams.append(self.Tc)
		c+=1	
	for camera in range(0,NumOCams):
		if camera<maincam:
			self.TransFromMainToOther.append(np.linalg.inv(self.allcams[camera][maincam-(camera+1)]))
			continue
		if camera==maincam:		
			for j in range(0,len(self.allcams[camera])):
				self.TransFromMainToOther.append(self.allcams[camera][j])
			break	



#solvepnp but we also set the projection amtrix the transformation matrixs and the rotation all in here oh and also print the cameras point position		
def SolvepnpTmatFrame1(self,mainCam,cameramatrice,NumOCams):
		rotationMat=[]
		OT34=[]
		del self.otherCamerasPositions[:]
		del self.othercameraProjMats[:]
		del self.oldTrans[:]
	
		rvec , tvec, inliers= cv2.solvePnPRansac(np.asarray(self.obj_points,dtype=np.float32), np.asarray(self.img_points,dtype=np.float32) , np.asarray(cameramatrice,dtype=np.float64),None,reprojectionError=15)
		rotationMat= cv2.Rodrigues(rvec)[0]
		cameraPos= -np.matrix(rotationMat).T*np.matrix(tvec)
		self.oldcamerapos=cameraPos
		Transform_CW= np.array([[rotationMat[0][0],rotationMat[0][1],rotationMat[0][2], tvec[0]],
					[rotationMat[1][0],rotationMat[1][1],rotationMat[1][2], tvec[1]],
					[rotationMat[2][0],rotationMat[2][1],rotationMat[2][2], tvec[2]],
					[0,0,0,1]],dtype=np.float32)
		self.oldTrans=Transform_CW
		Tm3x4=np.delete(Transform_CW,3,0)
		Proj3x4=np.dot(np.asarray(cameramatrice),Tm3x4)
	
		self.mainprojectionmatrixtoW=Proj3x4		
		self.cameraPos=cameraPos
		for i in range(0,len(self.TransFromMainToOther)):
			T=np.reshape(self.TransFromMainToOther[i],(4,4))
			self.othertrans=np.dot(T,Transform_CW)
			Rotationmatrix=np.array([[self.othertrans[0][0],self.othertrans[0][1],self.othertrans[0][2]],
					[self.othertrans[1][0],self.othertrans[1][1],self.othertrans[1][2]], 
					[self.othertrans[2][0],self.othertrans[2][1],self.othertrans[2][2]]],dtype=np.float32)
			Translatevec=np.array([[self.othertrans[0][3]],[self.othertrans[1][3]],[self.othertrans[2][3]]],dtype=np.float32)
			self.otherCamerasPositions.append(-np.matrix(np.asarray(Rotationmatrix,dtype=np.float32)).T*np.matrix(np.asarray(Translatevec,dtype=np.float32)))
			OT3x4=np.delete(self.othertrans,3,0)
			OT34.append(np.reshape(OT3x4,(3,4)))
		
		projections=0	
		for i in range(0,NumOCams):
			if i==mainCam:
				self.othercameraProjMats.append(Proj3x4)
			
				continue	
			self.othercameraProjMats.append(np.dot(np.asarray(self.calibdcams[i]),OT34[projections]))	
			projections+=1	
			

		k=0
		for i in range(0,NumOCams):
			if i==mainCam:
					print 'camera'+str(i+1)
					print self.cameraPos
					continue
			print 'camera'+str(i+1)
			print self.otherCamerasPositions[k]		
			k+=1
			
#exact same but the input for solvepnp is differen (we get it from mathingoldframesfor2d)
def SolvepnpTmatFrame2(self,mainCam,cameramatrice,NumOCams):
		rotationMat=[]
		Transform_CW=[]
		OT34=[]
		del self.otherCamerasPositions[:]
		del self.othercameraProjMats[:]
		self.rotationMat=[]
		tria3dpoints= np.reshape(np.asarray(self.neededtria,dtype=np.float32),(1,len(self.neededtria),3))
		rvec , tvec, inliers= cv2.solvePnPRansac(tria3dpoints[0], np.asarray(self.TwoDPAfterTria,dtype=np.float32) , np.asarray(cameramatrice,dtype=np.float64),None,reprojectionError=15)
		rotationMat= cv2.Rodrigues(rvec)[0]
		self.rotationMat=rotationMat
		x4=(math.atan2(rotationMat[2][1],rotationMat[2][2]))*(180/math.pi)
		y4=(math.atan2(-(rotationMat[2][0]),math.sqrt(rotationMat[2][1]*rotationMat[2][1]+rotationMat[2][2]*rotationMat[2][2])))*(180/math.pi)
		z4=(math.atan2(rotationMat[1][0],rotationMat[0][0]))*(180/math.pi)
		self.Rotationvec=[x4,y4,z4]
		cameraPos= -np.matrix(rotationMat).T*np.matrix(tvec)
		Transform_CW= np.array([[rotationMat[0][0],rotationMat[0][1],rotationMat[0][2], tvec[0]],
					[rotationMat[1][0],rotationMat[1][1],rotationMat[1][2], tvec[1]],
					[rotationMat[2][0],rotationMat[2][1],rotationMat[2][2], tvec[2]],
					[0,0,0,1]],dtype=np.float32)
		if abs(cameraPos[0]-self.oldcamerapos[0])>50 or abs(cameraPos[1]-self.oldcamerapos[1])>50	or abs(cameraPos[2]-self.oldcamerapos[2])>50:
			self.cameraPos=self.oldcamerapos
			Transform_CW=self.oldTrans
		else :
			self.cameraPos=cameraPos
			self.oldcamerapos=cameraPos
			self.oldTrans=Transform_CW
		Tm3x4=np.delete(Transform_CW,3,0)
		Proj3x4=np.dot(np.asarray(cameramatrice),Tm3x4)
		self.mainprojectionmatrixtoW=Proj3x4		
		for i in range(0,len(self.TransFromMainToOther)):
			T=np.reshape(self.TransFromMainToOther[i],(4,4))
			self.othertrans=np.dot(T,Transform_CW)
			Rotationmatrix=np.array([[self.othertrans[0][0],self.othertrans[0][1],self.othertrans[0][2]],
					[self.othertrans[1][0],self.othertrans[1][1],self.othertrans[1][2]], 
					[self.othertrans[2][0],self.othertrans[2][1],self.othertrans[2][2]]],dtype=np.float32)
			Translatevec=np.array([[self.othertrans[0][3]],[self.othertrans[1][3]],[self.othertrans[2][3]]],dtype=np.float32)
			self.otherCamerasPositions.append(-np.matrix(np.asarray(Rotationmatrix,dtype=np.float32)).T*np.matrix(np.asarray(Translatevec,dtype=np.float32)))
			OT3x4=np.delete(self.othertrans,3,0)
			OT34.append(np.reshape(OT3x4,(3,4)))
		projections=0	
		for i in range(0,NumOCams):
			if i==mainCam:
				self.othercameraProjMats.append(Proj3x4)
				continue	
			self.othercameraProjMats.append(np.dot(np.asarray(self.calibdcams[i]),OT34[projections]))	
			projections+=1
		k=0
		for i in range(0,NumOCams):
			if i==mainCam:
					print 'camera'+str(i+1)
					print self.cameraPos
					continue
			print 'camera'+str(i+1)
			print self.otherCamerasPositions[k]		
			k+=1
		self.TriaPoints3=[]
		self.listkp1=[]
def surfForAllcams(self,images):		
	self.descs=[]
	self.KPS=[]
	for i in range(0,len(images)):
		img=images[i]
		surf= cv2.SURF(100)
		kp, des=surf.detectAndCompute(img,None)
		self.descs.extend([des])
		self.KPS.extend([kp])
#just like the name says this function matches descriptors that we get from surf and whenever there is a match it triangulaates it and creates 3d points everything we need is saved in self.cam1matchingarray	
def matchingAndTriangulation(self,maincam,frames):
	c=1	
	#~ for i in range(0,numocam-1):
		#~ for j in range(c,numocam):

	maincam=maincam-1	
	del self.listkp1[:]
	del self.listkp2[:]
	del self.matchingkp[:]
	good=[]
	del self.gooddescs[:]
	if maincam==0:
		i=0
		j=1
	if maincam==1:
		i=1
		j=0
	des=self.descs[i]
	des2=self.descs[j]
	kp=self.KPS[i]
	kp2=self.KPS[j]	


	FLANN_INDEX_KDTREE=0
	index_params=dict(algorithm=FLANN_INDEX_KDTREE,trees=5)
	search_params= dict(checks=100)
	flann=cv2.FlannBasedMatcher(index_params,search_params)
#by using Brute force matching instead of FLANN we can get a less accurate results but the running time will be half of the time using FLANN
	#~ bf= cv2.BFMatcher()
	#~ matches = bf.knnMatch(des,des2, k=2)
	matches = flann.knnMatch(des,des2, k=2)
	for m,n in matches:
		if m.distance < 0.6*n.distance:
			good.append([m])
			img1_idx=m.queryIdx		
			img2_idx=m.trainIdx			
			self.gooddescs.append(des[img1_idx,:])
			self.matchingkp.append(kp[img1_idx])
			(x1,y1)=kp[img1_idx].pt
			(x2,y2)=kp2[img2_idx].pt
			self.listkp2.append((x2,y2))			
			tria4= cv2.triangulatePoints(np.asarray(self.othercameraProjMats[j]),np.asarray(self.mainprojectionmatrixtoW),np.asarray((x2,y2)),np.asarray((x1,y1)))
			Ntria4=tria4/tria4[3]
			tria3=np.delete(Ntria4,3,0)

			self.TriaPoints3.append([tria3])
		
	#~ draw_matches(frames[0], kp, frames[1], kp2, good, color=None,windowname='first frame')
	zipitPoints=zip(self.gooddescs,self.TriaPoints3,self.matchingkp)
	self.cam1matchingarray.extend(zipitPoints)
	

	

	
#here we see wich points we can see in current frame that we also saw in older FRAMES and we take these point from new frame and the 3d points from an older frame and use them in solvepnp2		
def matchingoldframesfor2d(self,maincam,frames):
	c=1	
	#~ for i in range(0,numocam-1):
		#~ for j in range(c,numocam):
	maincam=maincam-1
	self.TwoDPAfterTria=[]
	self.neededtria=[]
	good=[]
	des=self.descs[maincam]
	kp=self.KPS[maincam]
	des2=np.asarray([alldescs[0] for alldescs in self.cam1matchingarray],dtype=np.float32)
	FLANN_INDEX_KDTREE=0
	index_params=dict(algorithm=FLANN_INDEX_KDTREE,trees=5)
	search_params= dict(checks=300)
	flann=cv2.FlannBasedMatcher(index_params,search_params)
	matches = flann.knnMatch(des2,des, k=2)	
	#~ bf= cv2.BFMatcher()
	#~ matches = bf.knnMatch(des2,des, k=2)
	for m,n in matches:
		if m.distance < 0.4*n.distance:
			good.append([m])
			self.neededtria.append(np.asarray(self.cam1matchingarray)[m.queryIdx][1])
			self.TwoDPAfterTria.append(kp[m.trainIdx].pt)
			np.delete(des,m.trainIdx)
	#draw_matches(frames[maincam],[kp2[2] for kp2 in self.cam1matchingarray] , frames[maincam], kp, good, color=None,windowname='old frame compared to new one')
		
#bool function to find a chessboard pattern			
def foundchess(self,Image):
	
	SetpatternpointFrame1(self)
	img = cv2.cvtColor(Image, cv2.COLOR_BGR2GRAY)
	self.found, self.corners = cv2.findChessboardCorners(img, self.pattern_size)
	if self.found:
		term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
		cv2.cornerSubPix(img, self.corners, (5, 5), (-1, -1), term)
		self.img_points.append(self.corners.reshape(-1, 2).astype(np.float32))
		self.obj_points.append(self.pattern_points.astype(np.float32))	
		print 'ok' 
		return self.found 
	else:
		return self.found				
					
#setting pattern points like [0,0,0],[32,0,0].......
def SetpatternpointFrame1(self):
  self.pattern_points = np.zeros( (np.prod(self.pattern_size), 3), np.float32 )
  self.pattern_points[:,:2] = np.indices(self.pattern_size).T.reshape(-1, 2)
  self.pattern_points *= self.square_size
#saving all 3d POINTS DETECTED  
def save3dpoints(self,array,path):
	allpoints=np.asarray(array).tolist()
	j=0	
	#~ for i in range(0, len(allpoints)):
	points= {'final points':allpoints}   
	json.dump(points, codecs.open(path, 'w' , encoding = 'utf-8') , sort_keys = True , indent=4) 
	j=j+3	
	print 'data successfuly saved to '	


#draws surf points (only function that i didnt write i only used it for my testings)
def draw_matches(img1, kp1, img2, kp2, matches, color=None,windowname=''): 
	# We're drawing them side by side.  Get dimensions accordingly.
	# Handle both color and grayscale images.
	#~ print matches[1][0]
	if len(img1.shape) == 3:
		new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1], img1.shape[2])
	elif len(img1.shape) == 2:
		new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1])
	new_img = np.zeros(new_shape, type(img1.flat[0]))  
	# Place images onto the new image.
	new_img[0:img1.shape[0],0:img1.shape[1]] = img1
	new_img[0:img2.shape[0],img1.shape[1]:img1.shape[1]+img2.shape[1]] = img2
	
	# Draw lines between matches.  Make sure to offset kp coords in second image appropriately.
	r = 15
	thickness = 2
	if color:
		c = color
	for i in range(0,len(matches)):
		# Generate random color for RGB/BGR and grayscale images as needed.
		if not color: 
			c = np.random.randint(0,256,3) if len(img1.shape) == 3 else np.random.randint(0,256)
		
		# So the keypoint locs are stored as a tuple of floats.  cv2.line(), like most other things,
		# wants locs as a tuple of ints.
		
		end1 = tuple(np.round(kp1[matches[i][0].queryIdx].pt).astype(int))
		end2 = tuple(np.round(kp2[matches[i][0].trainIdx].pt).astype(int) + np.array([img1.shape[1], 0]))
		#~ cv2.line(new_img, end1, end2, c, thickness)
		cv2.circle(new_img, end1, r, c, thickness)
		cv2.circle(new_img, end2, r, c, thickness)
	
	plt.figure(windowname,figsize=(15,15))
	plt.imshow(new_img)
	plt.show(block=False)	
	plt.pause(5)
	plt.close()






