import numpy as np
import cv2
import os
from common import splitfn
import sys, getopt
from glob import glob
import codecs , json
import operator as op
import matplotlib.pyplot as plt
import math

class Calibration:
	def __init__(self,CameraId,pattern_size, square_size ):
		self.CameraId= CameraId
		self.pattern_size=pattern_size
		self.square_size= square_size
		self.img_mask= ''
		self.img_names={''}
		self.pattern_points=[]
		# 3d point in real world space
 		self.obj_points= []
 		# 2d points in image plane.      
		self.img_points=[]
		self.TranformationMats=[]
		self.triaPoints = []
		self.descs=[]
		self.KPS=[]
		self.calibdcams=[]
		self.Tc=[]
		self.Images=[]
		self.MATRICES=[]
		self.found=False
		self.TriaPoints3=[]
		self.NTriaPoints4=[]
		self.cameraPos=[]
		self.found=[]
		self.frames=[]
		self.CamsF=cv2.VideoCapture
		self.CamerasTransformationW=[]


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
def SetPatternPoints(self):
  self.pattern_points = np.zeros( (np.prod(self.pattern_size), 3), np.float32 )
  self.pattern_points[:,:2] = np.indices(self.pattern_size).T.reshape(-1, 2)
  self.pattern_points *= self.square_size
  #~ print self.pattern_points
#finds the chess corners and draws them on the actual image , pattern should be colorfull while original img colored grey 	
def FindDrawChess(self):
	for fn in self.img_names:
		print 'processing %s...' % fn,
		img = cv2.imread(fn, 0)
		self.h, self.w = img.shape[:2]
		found, self.corners = cv2.findChessboardCorners(img, self.pattern_size)
		if found:
			self.term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
			cv2.cornerSubPix(img, self.corners, (5, 5), (-1, -1), self.term)
		if not found:
			print 'chessboard not found'    
			continue
		self.img_points.append(self.corners.reshape(-1, 2).astype(np.float32))
		self.obj_points.append(self.pattern_points.astype(np.float32))

		for i in range(0,len(self.img_points[0])):
			plt.plot([self.img_points[0][i][0]], [self.img_points[0][i][1]], 'or')
		plt.show()
		print 'ok'  
    #this part is only to draw it doesnt change any of the results (optional)         
	#~ vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
	#~ cv2.drawChessboardCorners(vis, self.pattern_size, self.corners, found)
	#~ cv2.imshow('img', vis)
	#~ cv2.waitKey(5000)

	
def SolvepnpTmatFrame1(self,cameramatrice):
		self.lop=[]
		self.a=[]
		self.rotationMat=[]
		self.tvec=[]
		self.herllo=[]
		rvec , self.tvec, inliers= cv2.solvePnPRansac(np.asarray(self.obj_points,dtype=np.float32), np.asarray(self.img_points,dtype=np.float32) , np.asarray(cameramatrice,dtype=np.float64),None)


		self.rotationMat= cv2.Rodrigues(rvec)[0]
		cameraPos= -np.matrix(self.rotationMat).T*np.matrix(self.tvec)

		#~ print cameraPos
		self.a= np.array([[self.rotationMat[0][0],self.rotationMat[0][1],self.rotationMat[0][2], self.tvec[0]],
					[self.rotationMat[1][0],self.rotationMat[1][1],self.rotationMat[1][2], self.tvec[1]],
					[self.rotationMat[2][0],self.rotationMat[2][1],self.rotationMat[2][2], self.tvec[2]],
					[0,0,0,1]],dtype=np.float32)
					
		#~ print 'hard coded camera position from solvepnp on hard coded 3d points'
		#~ print self.a
		self.cameraPos.extend([cameraPos])
		JsonTransf=self.a.tolist()
		self.TranformationMats.extend([JsonTransf])
		self.herllo=np.delete(self.a,3,0)
		self.lop=np.dot(cameramatrice,self.herllo)
		x4=(math.atan2(self.rotationMat[2][1],self.rotationMat[2][2]))*(180/math.pi)
		y4=(math.atan2(-(self.rotationMat[2][0]),math.sqrt(self.rotationMat[2][1]*self.rotationMat[2][1]+self.rotationMat[2][2]*self.rotationMat[2][2])))*(180/math.pi)
		z4=(math.atan2(self.rotationMat[1][0],self.rotationMat[0][0]))*(180/math.pi)
	

		print'self.a'
		print self.a
		print'projection'
		print self.lop
		#~ print'please'
		print 'hard coded cameraPos'
		print cameraPos
		print 'hard codedRotationvec'
		print (x4,y4,z4)
		print 'xxxxxxxxxxxxxxxxxx'
