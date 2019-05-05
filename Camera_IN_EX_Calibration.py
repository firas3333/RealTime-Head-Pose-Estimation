import MainClass
import cv2
import codecs , json
import numpy as np
import itertools
from glob import glob
def CalibrateCams(ExtrinsicsOnly):	
	NumOcams=3
	CamerasArray=[]
	TranformationM=[]
	calib= MainClass.Calibration()
	Cobj=calib.obj_points
	Cimg=calib.img_points
#loading details of the calibration you want to make (square size ,pattern size , cameraId)	
	MainClass.getcalibrationdetails(calib,PathOfCalibDetails)
	if not ExtrinsicsOnly:
#intrinsic		
		i_P_S=0
		for i in range(0,NumOcams):
#load images from your file for calibration(please notice its written in the internet that 10 imgs are gooenough for intrinsics but i found out that i need arround 30 images for good calibration)			
			MainClass.GetImages(calib,i+1,PathToIntrinsicsImages)
#as the name suggests these 2 functions set the pattern find the chess and show on the screen the image and the pattern o it 						 
			MainClass.SetPatternPoints(calib,i_P_S)
			MainClass.FindDrawChess(calib,i_P_S)
#image ratio			
			w=calib.w
			h=calib.h
#this magic function does all the calculation on the images and returns the intrinsic matrix plus distortion coef (i save the distortion even though there is no need)			
			rms, camera_matrix, dist_coefs, rvec, tvec = cv2.calibrateCamera(Cobj, Cimg, (w, h))
#must change the format of the data to a list sso we can save it in json file			
			cameraMat_list= camera_matrix.tolist()
			dist_coefs_List = dist_coefs.tolist()
			CamerasArray.extend([ calib.CalibDetails[i_P_S] , cameraMat_list , dist_coefs_List])
			i_P_S+=3	
			del calib.obj_points[:]
			del calib.img_points[:]
#saving a json file for each cam and its matrix			
		MainClass.SaveCalibratedCams(calib,CamerasArray,CamMatSaveDestination)
#only extrinsic calib
	i_P_S=0
	for i in range(0,NumOcams):
#loading intrinsic matrix		
		MainClass.GetCamsMatrix(calib,NumOcams)
#loading and changing the format of the image so we can do proccesing on it				
		calib.img_mask = PathToExtrinsicsImages+'/cam'+str(i+1)+'.jpg'
		calib.img_names = glob(calib.img_mask)
		MainClass.SetPatternPoints(calib,i_P_S)
		MainClass.FindDrawChess(calib,i_P_S)
#solvepnp to get for each cam Tcw		
		MainClass.Solvepnpforixtrinsic(calib,calib.calibdcams[i])
		i_P_S+=3
	MainClass.savingTM_Rotate_trnaslate(calib,NumOcams,calib.TranformationMats)
	cv2.destroyAllWindows()	

if __name__ == "__main__":
	#~ #if you want to do only extrinsic calibration ExtrinsicsOnly=True for both intrinsic and extrinsic ExtrinsicsOnly=False
	ExtrinsicsOnly=True
	PathOfCalibDetails='/home/odroid/Firass Project IO Data/calibrationdetails.txt'
	PathToIntrinsicsImages='/home/odroid/Firass Project IO Data/Images for intrisics/Camera'
	PathToExtrinsicsImages='/home/odroid/Firass Project IO Data/Images for extirinsic calib'
	CamMatSaveDestination="/home/odroid/Firass Project IO Data/Calibration Output Data/Camera" 
	CalibrateCams(ExtrinsicsOnly)

