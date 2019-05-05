import cv2
import numpy as np
def Calibration(ForIntrnsic,ForExtrinsic):
	if ForExtrinsic:
		cam = cv2.VideoCapture(0)
		cam2 = cv2.VideoCapture(1)
		#~ cam3 = cv2.VideoCapture(2)
		img_counter = 1
		numOfCam=1
		i=1
		while True:
			ret2, frame2 = cam2.read()
			cv2.imshow("cam2", frame2)	
			
			
			ret, frame = cam.read()
			cv2.imshow("cam1", frame)


			#~ ret, frame3 = cam3.read()
			#~ cv2.imshow("cam3", frame3)

			if not ret:
				break
			k = cv2.waitKey(1)

			if k%256 == 27:
				# ESC pressed
				print("Escape hit, closing...")
				break
			elif k%256 == 32:
				# SPACE pressed
				img_name = Cam1Extrinsic   
				#~ img_name5 = "/home/odroid/imgs/for extirinsic calib/cam31.jpg"     
				img_name3 = Cam2Extrinsic   
				cv2.imwrite(img_name, frame)
				cv2.imwrite(img_name3, frame2)
				#~ cv2.imwrite(img_name5, frame3)
				print("{} written!".format(img_name))
				img_counter += 1
		frame2=[]
		frame=[]
		cam2.release()		
		cam.release()


		cv2.destroyAllWindows()




	elif ForIntrnsic:
		camera=0
		Cam = cv2.VideoCapture(camera)
		img_counter = 1
		while True:
		 ret, frame = Cam.read()
		 cv2.imshow("cam"+str(camera+1), frame)
		 if not ret:
			 break
		 k = cv2.waitKey(1)
		 if k%256 == 27:
			 # ESC pressed
			 print("Escape hit, closing...")
			 break
		 elif k%256 == 32:
			 # SPACE pressed
			 img_name = CamPicForIntrinsic+'/Camera'+str(camera+1)+"/calib"+str(img_counter)+".jpg"     
			 cv2.imwrite(img_name, frame)
			 print("{} written!".format(img_name))
			 img_counter += 1
		cv2.waitKey(30)
		cam.release()
		cv2.destroyAllWindows()
if __name__ == "__main__":	

	#Images save destinations
	CamForIntrinsic='/home/odroid/Firass Project IO Data/Images for intrisics'
	Cam1Extrinsic="/home/odroid/Firass Project IO Data/Images for extirinsic calib/cam1.jpg"  
	Cam2Extrinsic="/home/odroid/Firass Project IO Data/Images for extirinsic calib/cam2.jpg"  
	#True= taking images for exrinsic calibration , False= images for intrinsic
	ForIntrnsic=False
	ForExtrinsic=True
	Calibration(ForIntrnsic,ForExtrinsic)

