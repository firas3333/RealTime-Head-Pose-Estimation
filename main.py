import MainClass
import cv2
import numpy as np
import datetime
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
import math
import itertools
#number of cameras connected to your pc
def Run(Path,NumOCams,Plot,PlotRotation,Plot3dPoints,Plot3dAndCamposition, PlotOnlyCamPos):
	fig = plt.figure(1)
	fig2 = plt.figure(2)
	fig3 = plt.figure(3)

	oldframe=[]
	all3dPoints=[]
	Plotpoint=[]
	a=0
	frames=[]
	cams=[]
	Maincampos=[]
	#openning camera using threading
	vs = MainClass.WebcamVideoStream(src=0).start()
	vs2 = MainClass.WebcamVideoStream(src=1).start()
	#creat our object
	calib= MainClass.Calibration()
	#get the square size the pattern size and the chessboard size from same file we used for the calibration
	MainClass.getcalibrationdetails(calib,Path)
	#loading all cameras intrinsic matrix into an array calib.calibdcams 
	MainClass.GetCamsMatrix(calib,NumOCams)
	#this loop we use only once at the start of the running until we find chessboard pattern for the first time , we need the pattern because without it we still dont have 3d and 2d points (just like in the explination file sayed)
	while a==0:
		#~ start = datetime.datetime.now()
		del frames[:]
	#reading 2 frames each itteration and saving them in an array 
		frame2 = vs2.read()
		frame = vs.read()
		frames=[frame,frame2]
	#we check both frames in hopes to find the chess 
		for i in range(0,len(frames)):
			#~ start = datetime.datetime.now()		
	#foundchess() returns bool=calib.found if it finds pattern using cv2.findChessboardCorners	
			MainClass.foundchess(calib,frames[i])
			if calib.found:
				#~ end = datetime.datetime.now()
				#~ print 'finding'+str( (end-start).total_seconds())
	#The main camera fo the rest of the running time is the one who found the chess			
				mainCam=i+1
				#~ print 'hello im main camera '+str(mainCam)
	#	GetTansMats(self,index of mainCam,NumOCams) using transformation matrixes logic returns the transformation matrix from main cam to all other cams				
				MainClass.GetTansMats(calib,i,NumOCams)
	#setting the intrinsic matrix for the mainCamera			
				cameramatrice=calib.calibdcams[i]
				#~ start = datetime.datetime.now()
				#~ print cameramatrice
	#	SolvepnpTmatFrame1 does solvepnp on 3dpoints=the chessboard point that we set and points are the chessboard pattern that we found in this frame. Returns cameras Projections, Translations , Rotations		
				MainClass.SolvepnpTmatFrame1(calib,i,cameramatrice,NumOCams)
				#~ end = datetime.datetime.now()
				#~ print (end-start).total_seconds()
	#does surf on all frames and saves each cams des and kp in self.descs self.KPS	
				MainClass.oldframe=frames
				
				MainClass.surfForAllcams(calib,frames)
	#Matches all the keypoints the we found for each couple of frames Then performs a triangulate on these points and creates self.cam1matchingarray= zip(descs,3dpoints,2dpoints),its the 2d points and descs ,that came from mainCam, that we used to triangulate points   			
				MainClass.matchingAndTriangulation(calib,mainCam,frames)
				a+=1
				#~ end = datetime.datetime.now()
				#~ print (end-start).total_seconds()
				
				break	
			else:
				continue
	#i found out that its best to clean some of self.cam1matchingarray every 6 seconds it just gave me best results 		
	counting6Seconds=0	
	try:	
					
		while True:
			print '---------------------------------------------------------'
			start1 = datetime.datetime.now()
			del frames[:]
			frame2 = vs2.read()
			frame = vs.read()
			frames=[frame,frame2]
		#just like in first frame	
			#~ start = datetime.datetime.now()

			MainClass.surfForAllcams(calib,frames)
			#~ end1 = datetime.datetime.now()
			#~ totalTime1=(end1-start).total_seconds()
			#~ print 'Surf both cameras total time'+str(totalTime1)
		#now that i have 2d points that i can see from this frame and 2d points from last frame that has 3d points i match the 2d points and take the 3d points of the good matches from  self.cam1matchingarray and the 2d points from this fram that match
			#~ start = datetime.datetime.now()

			MainClass.matchingoldframesfor2d(calib,mainCam,frames)
			#~ end1 = datetime.datetime.now()
			#~ totalTime1=(end1-start).total_seconds()
			#~ print 'matchingoldframesfor2d'+str(totalTime1)
		#i do solvepnp for the same output but this time the input to solvepnp is : 3d=self.neededtria 2d=self.TwoDPAfterTria wich i got from the function above	
			#~ start = datetime.datetime.now()


			MainClass.SolvepnpTmatFrame2(calib,mainCam-1,cameramatrice,NumOCams)
			#~ end1 = datetime.datetime.now()
			#~ totalTime1=(end1-start).total_seconds()
			#~ print 'pnpFRame 2'+str(totalTime1)
		#Now we need to match 2d points from our cameras on this frame and triangulate the points to ready up the 3d points for the next frame	
			#~ start = datetime.datetime.now()

			MainClass.matchingAndTriangulation(calib,mainCam,frames)
			#~ end1 = datetime.datetime.now()
			#~ totalTime1=(end1-start).total_seconds()
			#~ print 'matchingAndTriangulation total time'+str(totalTime1)
		#calculating if 6 seconds has passed since the last clean up	

			end = datetime.datetime.now()
			totalTime=(end-start1).total_seconds()
			print totalTime
			counting6Seconds+=totalTime
			needed3d=calib.neededtria
			if counting6Seconds>=6:
				print type(calib.cam1matchingarray)
				for i in range(0,len(calib.cam1matchingarray)):
					all3dPoints.append(calib.cam1matchingarray[i][1])
				calib.cam1matchingarray=calib.cam1matchingarray[:60]
				#~ calib.cam1matchingarray=calib.cam1matchingarray[:30]+calib.cam1matchingarray[len(calib.cam1matchingarray)-30:]
				counting6Seconds=0
			#~ print 'proccessing time on last frame was : '+str(totalTime)
	#this section of code is only for plotting data but it slows the code drasticaly --------------------------------------------------------------
			#~ start = datetime.datetime.now()
			
			
			if Plot:
			#mainCam Rotation	
				Rx=np.dot(calib.rotationMat,[1,0,0])
				Ry=np.dot(calib.rotationMat,[0,1,0])
				Rz=np.dot(calib.rotationMat,[0,0,1])	
			#all cameras position		
				Maincampos=calib.cameraPos
				Cam2Pos=calib.otherCamerasPositions[0]
				Cam2=Cam2Pos
			#origin point to plot rotation of the mainCam	
				origin=[0,0,0]
				
				
				if PlotRotation:
					plt.figure(1)
					plt.cla()
					ax = fig.add_subplot(111, projection='3d')
					ax.view_init(elev=-85.,azim=-179.)

					ax.set_xlabel('x')
					ax.set_ylabel('y')
					ax.set_zlabel('z')
					ax.set_title('MAIN CAMERA ROTATION')
				#view 1: uncomment for plotting the rotation	
					ax.set_xlim3d(-1,1)
					ax.set_ylim3d(0,1)
					ax.set_zlim3d(0,1)					
					ax.plot([ float(origin[0]),float(Rz[0])], [float(origin[1]),float(Rz[1])], [float(origin[2]),float(Rz[2])],marker='^')
					ax.plot([ float(origin[0]),float(Rx[0])], [float(origin[1]),float(Rx[1])], [float(origin[2]),float(Rx[2])],marker='^')
					ax.plot([ float(origin[0]),float(Ry[0])], [float(origin[1]),float(Ry[1])], [float(origin[2]),float(Ry[2])],marker='^')
					
			
			#view 2: uncomment for 3d points 
				if Plot3dPoints:	
					plt.figure(2)
					plt.cla()
					ax2 = fig2.add_subplot(111, projection='3d')
					ax2.view_init(elev=-85.,azim=-179.)

					ax2.set_xlabel('x')
					ax2.set_ylabel('y')
					
					ax2.set_zlabel('z')
					ax2.set_title('3D POINTS red Cameras blue')
					ax2.set_xlim3d(-800,800)
					ax2.set_ylim3d(-800,800)
					ax2.set_zlim3d(-3000,3000)
					for i in range(0,len(needed3d)):
						bobo=0
						plt.plot([ needed3d[i][0][bobo][0]], [needed3d[i][0][bobo+1][0]], [needed3d[i][0][bobo+2][0]], 'or')
					if Plot3dAndCamposition:
						plt.plot([ float(Maincampos[0])], [float(Maincampos[1])], [float(Maincampos[2])],'or',color='b')
						plt.plot([ float(Cam2[0])], [float(Cam2[1])], [float(Cam2[2])],'or',color='b')

		
			#view 3: uncomment for cameras position 
				if PlotOnlyCamPos:
					plt.figure(3)
					plt.cla()
					ax3= fig3.add_subplot(111, projection='3d')
					ax3.view_init(elev=-85.,azim=-179.)

					ax3.set_xlabel('x')
					ax3.set_ylabel('y')
					ax3.set_zlabel('z')
					ax3.set_xlim3d(-300,300)
					ax3.set_ylim3d(-300,300)
					ax3.set_zlim3d(-3000,3000)
				#uncomment this part only and view 2  to view the cameras position and 3d points		
					plt.plot([ float(Maincampos[0])], [float(Maincampos[1])], [float(Maincampos[2])],'or',color='b')
					plt.plot([ float(Cam2[0])], [float(Cam2[1])], [float(Cam2[2])],'or',color='r')
				#~ plt.show(block=False)
				plt.draw()
				plt.pause(0.01)
				#~ plt.close('all')	
			#~ plt.show(block=False)	
				
			#~ end1 = datetime.datetime.now()
			#~ totalTime1=(end1-start).total_seconds()
			#~ print 'Ploto'+str(totalTime1)	
	# press ctrl+c to exist the code, save all the 3d points in a json file and plot the first 300 of them		
	except KeyboardInterrupt:
		finalizedpoints=[]
		cv2.destroyAllWindows()
		finalPoints=np.asarray(all3dPoints).reshape(len(all3dPoints), 3)
		print len(finalPoints)
		newr=[tuple(row) for row in finalPoints]
		finalPoints=np.unique(newr)
		


		for i in range(0,len(finalPoints)):
				if not ( (-3000>=finalPoints[i][0] or finalPoints[i][0]>=3000) or (-3000>=finalPoints[i][1] or finalPoints[i][1]>=3000) or (-4000>=finalPoints[i][2] or finalPoints[i][2]>=4000) ):
						finalizedpoints.append(finalPoints[i])
						
		print len(finalizedpoints)
				
		MainClass.save3dpoints(calib,finalizedpoints,pathfor3dpoints)			
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		
		ax.view_init(elev=-85.,azim=-179.)
		ax.set_xlabel('x')
		ax.set_ylabel('y')
		ax.set_zlabel('z')
		ax.set_xlim3d(-1500,1500)
		ax.set_ylim3d(-1500,1500)
		ax.set_zlim3d(-3000,3000)
		for i in range(0,300):
			plt.plot([ finalizedpoints[i][0]], [finalizedpoints[i][1]], [finalizedpoints[i][2]], 'or')					
		print len(finalizedpoints)
		plt.show(block=False)	
if __name__ == "__main__":	
	pathfor3dpoints="/home/odroid/Firass Project IO Data/Calibration Output Data/3dPoints.json"
	PathForData='/home/odroid/Desktop/img/calibrationdetails.txt'
	NumOCams=2
#For plotting drastically slows running time
	Plot=False
	PlotRotation=False
	Plot3dPoints=False
	Plot3dAndCamposition=False
	PlotOnlyCamPos=False
	
	Run(PathForData,NumOCams,Plot,PlotRotation,Plot3dPoints,Plot3dAndCamposition,PlotOnlyCamPos)
	
