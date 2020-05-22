#=============================================================================
"""
Code Information:
	Programmer: Eng. John Betancourt G.
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer Vision Team

Description:
	This class merge 3 image from video streams to get a panoramic stitched 
	imaged, the class only calculates once the stitcher class variable, then 
	the process is fast enough for real time stitching or video streaming of 
	result image. If there are already projection parameters for wach video 
	streaming camera, then calculate the equivalent projections to work in the
	stitched space

Version: 
	1.4

Merits to:
	Real-time panorama and image stitching with OpenCV
	Adrian Rosebrock 
	January 25, 2016 
	https://www.pyimagesearch.com/2016/01/25/real-time-panorama-and-image-stitc
	hing-with-opencv/
"""

# =============================================================================
import numpy as np
import copy
import math
import cv2
import os

from vision.utils.vision_utils import printlog

# =============================================================================
class Stitcher():
	
	def __init__(self, abs_path="", super_stitcher=False):
		"""	class constructor that merge 3 image from video streams to get a 
			panoramic stitched imaged, the class only calculates once the 
			stitcher class variable, then the process is fast enough for real 
			time stitching or video streaming of result image
		Args:
			super_stitcher: `boolean`  Enables or disables super stitching mode 
				this means smooth changes and application of ROI
		Returns:
		"""

		super(Stitcher, self).__init__() # Initialize inherited components

		# ---------------------------------------------------------------------
		# check if we are using OpenCV v3.X and initialize the
		# cached homography matrix
		self.isv3 = is_cv3()

		# General variables
		self.pano_size = None # Size of panoramic - (type) = (width, height)
		self.warp_flags = int(cv2.INTER_NEAREST) # Scaling factor to process
		self.Window_size = (640, 360) # Size to get segment of stitched video
		self.stitch_mode = [0, 0, 0] # Definition of stitching mode [L, C, R]
		self.vir_pan_val = 0 # Virtual pan value

		# Enables or disables super stitching mode: smooth changes and application
		# of ROI (Region Of Interest)
		self.super_mode = super_stitcher

		# ---------------------------------------------------------------------
		# CENTER (C) + RIGHT (R) = (CR)
		self.cachedHCR     = None # Spacial Transformation matrix for C and R images
		self.cachedHCR_INV = None # Spacial Transformation inverse matrix for C and R images
		self.y_offsetCR    = None # Y offset in stitching (int) for C and R
		self.warp_CR_size  = None # Size of first stitching  - (type) = (width, height)
		self.cachedMCR 	   = None # Final transformation matrix in pair images C and R for super mode
		self.cachedMCR_INV = None # Final inverse transformation matrix in pair images C and R for super mode
		self.pano_size_CR  = None # Panoramic size for RC stitching configuration
		self.out_limitsCR 	  = None # out limits of panoramic stitching 
		self.in_limitsCR 	  = None # out limits of panoramic stitching 
		self.x_coord_CR_limit = None # X coordinate to overlay transparency in pair images C and R
		self.alpha_mask_CR 	  = None # Alpha mask 1 to over lay transparency in pair images C and R
		self.alpha_mask_CR2   = None # Alpha mask 2 to over lay transparency in pair images C and R

		# ---------------------------------------------------------------------
		# LEFT (L) + CENTER (C) = (LC)

		self.cachedHLC     = None # Spacial Transformation matrix for L and C images
		self.cachedHLC_INV = None # Spacial Transformation inverse matrix for L and C images
		self.warp_LC_size  = None # Size of first stitching  - (type) = (width, height)
		self.out_limitsLC  = None # OUT limits of panoramic stitching in LC Configuration
		self.in_limitsLC   = None # in limits of panoramic stitching in LC Configuration
		self.y_offsetLC    = None # Y offset in stitching (int) in LC Configuration
		self.cachedMLC 	   = None # Final transformation matrix in pair images L and C for super mode
		self.cachedMLC_INV = None # Final inverse transformation matrix in pair images L and C for super mode
		self.pano_size_LC  = None # Panoramic size for RC stitching configuration
		self.x_coord_LC_limit 	 = None # X coordinate to overlay transparency in pair images L and C
		self.alpha_mask_LC 		 = None # Alpha mask 1 to over lay transparency in pair images L and C
		self.alpha_mask_LC2 	 = None # Alpha mask 2 to over lay transparency in pair images L and C
		self.source_C_LC_corners = [] # Corners location in panoramic stitching for C image source in LC Configuration
		self.source_L_LC_corners = [] # Corners location in panoramic stitching for L image source in LC Configuration

		# ---------------------------------------------------------------------
		# LEFT (L) + CENTER&RIGHT (C&R) = (LC&R)

		self.cachedHLCR     = None # Spacial Transformation matrix for L and CR images
		self.cachedHLCR_INV = None # Spacial Inverse Transformation matrix for L and CR images 
		self.cachedMLCR     = None # Transformation matrix for final perspective correction in LCR Configuration
		self.cachedMLCR_INV = None # Transformation Inverse matrix for final perspective correction in LCR Configuration
		self.y_offsetLCR    = None # Y offset in stitching (int) in LCR Configuration
		self.pano_size_LCR  = None # Panoramic size for LCR stitching configuration
		self.warp_size_LCR  = None # Size of final stiching  - (typle) = (withd, height) in LCR Configuration
		self.out_limitsLCR 	  = None # out limits of panoramic stitching in LCR Configuration
		self.in_limitsLCR 	  = None # in limits of panoramic stitching in LCR Configuration
		self.source_R_corners = None # Corners location in panoramic stitching for R image source in LCR Configuration
		self.source_L_corners = None # Corners location in panoramic stitching for L image source in LCR Configuration
		self.source_C_corners = None # Corners location in panoramic stitching for C image source in LCR Configuration
		self.x_coord_LCR_limit 	= None # X coordinate to overlay transparency in pair images L and CR
		self.alpha_mask_LCR 	= None # Alpha mask 1 to over lay transparency in pair images L and CR
		self.alpha_mask_LCR2 	= None # Alpha mask 2 to over lay transparency in pair images L and CR
		self.corners = [] 			   # Corners of all images
		
		# ---------------------------------------------------------------------
		# Projection parameters for video streamings
		self.PP_CntL = None # Left   original contour in distortion space
		self.PP_CntC = None # Center original contour in distortion space
		self.PP_CntR = None # Right  original contour in distortion space
		self.LC_CtnL  = None # L + C -> Left contour surface in stitching space
		self.LC_CtnC  = None # L + C -> Center contour surface in stitching space
		self.CR_CtnC  = None # C + R -> Center contour surface in stitching space
		self.CR_CtnR  = None # C + R -> Right contour surface in stitching space
		self.LCR_CtnL = None # L + C + R -> Left contour surface in stitching space
		self.LCR_CtnC = None # L + C + R -> Center contour surface in stitching space
		self.LCR_CtnR = None # L + C + R -> Right contour surface in stitching space
		self.LC_CtnL_Mc  = None # L + C -> Mass center of Left contour surface in stitching space
		self.LC_CtnC_Mc  = None # L + C -> Mass center of Center contour surface in stitching space
		self.CR_CtnC_Mc  = None # C + R -> Mass center of Center contour surface in stitching space
		self.CR_CtnR_Mc  = None # C + R -> Mass center of Right contour surface in stitching space
		self.LCR_CtnL_Mc = None # L + C + R -> Mass center of Left contour surface in stitching space
		self.LCR_CtnC_Mc = None # L + C + R -> Mass center of Center contour surface in stitching space
		self.LCR_CtnR_Mc = None # L + C + R -> Mass center of Right contour surface in stitching space

		# ---------------------------------------------------------------------
		# Center of original images in stitching image
		self.LCR_Lc = None # L + C + R -> Center of left original image in stitching image
		self.LCR_Cc = None # L + C + R -> Center of Center original image in stitching image
		self.LCR_Rc = None # L + C + R -> Center of Right original image in stitching image
		self.LC_Lc = None  # L + C -> Center of left original image in stitching image
		self.LC_Cc = None  # L + C -> Center of Center original image in stitching image
		self.CR_Rc = None  # C + R -> Center of Right original image in stitching image
		self.CR_Cc = None  # C + R -> Center of Center original image in stitching image

		# ---------------------------------------------------------------------
		# Load default stitcher configuration for triple video streaming
		self.path_config_file = abs_path if abs_path != "" else "./default_config.npz"
		self.load_stitcher(source_path=self.path_config_file)

	def calibrate_stitcher(self, images, ratio=0.75, reprojThresh=4.0):
		""" If stitcher is not initialized yet, this function will do it and 
			then it calculates the panoramic stitching of A B and C images 
		Args:
			images: `cv2.mat list` list of images in order left center right
			ratio: 'float' aspect ratio for panoramic result (Keep it in 0.75)
			reprojThresh: 'float' correlation value for desired matched points 
		Returns:
			result_CAB: 'cv2.mat' result of stitching A, B, and C images into a 
			panoramic image
		"""

		# --------------------------------------------------------------------
		# unpack the images
		# Right, Center, Left -> Streaming
		(imageL, imageC, imageR) = images

		# Process variables
		save_file = False

		# ---------------------------------------------------------------------
		# if the cached homography matrix is None for pair L/C, then we need to
		# apply key-point matching to construct it
		if self.cachedHLC is None:

			# Process variables
			
			printlog(msg="Stitcher calibration process started", msg_type="INFO")
			save_file = True

			# Size of first stitching
			self.warp_LC_size = (
				int(imageL.shape[1] + imageL.shape[1]), 
				int(imageL.shape[0] + imageL.shape[0]))

			# detect key-points and extract
			(kpsA, featuresA) = self.detectAndDescribe(image = imageC)
			(kpsB, featuresB) = self.detectAndDescribe(image = imageL)
			
			# match features between the two images
			MLC = self.matchKeypoints(
				kpsA = kpsA, kpsB = kpsB,
				featuresA = featuresA, featuresB = featuresB, 
				ratio = ratio, reprojThresh = reprojThresh)

			# if the match is None, then there aren't enough matched
			# key-points to create a panorama
			if MLC is None:
				printlog(msg="Matched no possible for pair center and left images", 
					msg_type="ERROR")
				return None
			else:
				printlog(msg="Pair left and center images matched successfully", 
					msg_type="INFO")

			# cache the homography matrix
			self.cachedHLC = MLC[1]

			self.corners_LC = []
			self.corners_LC.append(np.array(get_projection_point_dst((0, 0, 1), self.cachedHLC)))
			self.corners_LC.append(np.array(get_projection_point_dst((0, imageC.shape[0], 1), self.cachedHLC)))
			self.corners_LC.append(np.array(get_projection_point_dst((imageC.shape[1], 0, 1), self.cachedHLC)))
			self.corners_LC.append(np.array(get_projection_point_dst((imageC.shape[1], imageC.shape[0], 1), self.cachedHLC)))
			self.corners_LC.append(np.array((0, 0)))
			self.corners_LC.append(np.array((imageL.shape[1], imageL.shape[0])))
			self.corners_LC.append(np.array((0, imageL.shape[0])))
			self.corners_LC.append(np.array((imageL.shape[1], 0)))

			self.out_limitsLC, self.in_limitsLC, corners = \
				get_extreme_contour_corners(self.corners_LC)
			self.y_offsetLC = self.out_limitsLC[0]
			self.cachedHLC[1][2] -= self.y_offsetLC
			self.cachedHLC_INV = np.linalg.inv(self.cachedHLC)

			for idx, corner in enumerate(self.corners_LC):
				self.corners_LC[idx][1] = corner[1] - self.y_offsetLC

			self.source_C_LC_corners = self.corners_LC[0:4]
			self.source_L_LC_corners = self.corners_LC[4:8]

		# Find first point of L image in C image
		if self.x_coord_LC_limit is None:
			dmax = max(imageC.shape[:2])
			for pt in self.source_C_LC_corners:
				if pt[0] < imageL.shape[1]:
					d = imageC.shape[1] - pt[0]
					if d < dmax:
						dmax = d
						self.x_coord_LC_limit = int(pt[0])
			mask_size =  int(imageL.shape[1] - self.x_coord_LC_limit)
			alpha_mask_LC = np.zeros((imageL.shape[0], mask_size, 3), np.float32)
			for x_coord in range(0, mask_size):
				alpha_mask_LC[0:alpha_mask_LC.shape[0], x_coord] = \
					float(x_coord)/float(mask_size)
			self.alpha_mask_LC = cv2.flip(alpha_mask_LC[::], 1)
			self.alpha_mask_LC2 = np.ones((imageL.shape[0], mask_size, 3), np.float32) - self.alpha_mask_LC

		# apply a perspective transform to stitch the images together
		result_LC = cv2.warpPerspective(imageC, self.cachedHLC, self.warp_LC_size)

		# Process super mode
		if self.super_mode:
			Ca = imageL[:, self.x_coord_LC_limit:]/255.
			Cb = result_LC[
				self.source_L_LC_corners[0][1]: self.source_L_LC_corners[2][1], 
				self.x_coord_LC_limit  	     : imageL.shape[1]]/255. 
			Cr = (Ca*self.alpha_mask_LC + Cb*self.alpha_mask_LC2)*255

		# Overlay the center image to match
		result_LC[self.source_L_LC_corners[0][1]: self.source_L_LC_corners[2][1], 
			0					            : self.source_L_LC_corners[1][0]] = imageL

		# Process super mode
		if self.super_mode:
			result_LC[self.source_L_LC_corners[0][1]: self.source_L_LC_corners[2][1], 
					self.x_coord_LC_limit         : imageL.shape[1]] = Cr

		# Select ROI of first panoramic stitching 
		result_LC = result_LC[0:self.out_limitsLC[1] - self.y_offsetLC, 
							0:self.out_limitsLC[3]]

		# Assign panoramic size
		self.pano_size_LC = (result_LC.shape[1], result_LC.shape[0])

		# Correct result perspective
		if self.cachedMLC is None:
			# Calculate rotation matrix from surface from original source image to 
			# projected four points surfaces
			src_points = np.array( corners, dtype=np.float32) 
			dst_points = np.array([[0                , 0               ], 
								[result_LC.shape[1] , 0               ], 
								[0                , result_LC.shape[0]],
								[result_LC.shape[1] , result_LC.shape[0]]],dtype=np.float32)
			self.cachedMLC = cv2.getPerspectiveTransform(src_points, dst_points)
			self.cachedMLC_INV = np.linalg.inv(self.cachedMLC)

		# Final adjustment - apply final transform
		result_LC = cv2.warpPerspective(src = result_LC, 
										M = self.cachedMLC, 
										dsize = self.pano_size_LC)

		self.pano_size_LC = (result_LC.shape[1], result_LC.shape[0])

		# --------------------------------------------------------------------
		# if the cached homography matrix is None for pair R/C, then we need to
		# apply key-point matching to construct it
		if self.cachedHCR is None:

			save_file = True

			# Size of first stitching
			self.warp_CR_size = (
				int(imageR.shape[1] + imageC.shape[1]), 
				int(imageR.shape[0] + imageC.shape[0]))

			# detect key-points and extract
			(kpsA, featuresA) = self.detectAndDescribe(image = imageR)
			(kpsB, featuresB) = self.detectAndDescribe(image = imageC)
			
			# match features between the two images
			MCR = self.matchKeypoints(
				kpsA = kpsA, kpsB = kpsB,
				featuresA = featuresA, featuresB = featuresB, 
				ratio = ratio, reprojThresh = reprojThresh)

			# if the match is None, then there aren't enough matched
			# key-points to create a panorama
			if MCR is None:
				printlog(msg="Matched no possible for pair center and left video", 
					msg_type="ERROR")
				return None
			else:
				printlog(msg="Pair center and left images matched successfully", 
					msg_type="INFO")

			# cache the homography matrix
			self.cachedHCR = MCR[1]

			self.corners.append(np.array(get_projection_point_dst((0, 0, 1), self.cachedHCR)))
			self.corners.append(np.array(get_projection_point_dst((0, imageR.shape[0], 1), self.cachedHCR)))
			self.corners.append(np.array(get_projection_point_dst((imageR.shape[1], 0, 1), self.cachedHCR)))
			self.corners.append(np.array(get_projection_point_dst((imageR.shape[1], imageR.shape[0], 1), self.cachedHCR)))
			self.corners.append(np.array((0, 0)))
			self.corners.append(np.array((imageC.shape[1], imageC.shape[0])))
			self.corners.append(np.array((0, imageC.shape[0])))
			self.corners.append(np.array((imageC.shape[1], 0)))

			self.out_limitsCR, self.in_limitsCR, corners = \
				get_extreme_contour_corners(self.corners)
			self.y_offsetCR = self.out_limitsCR[0]
			self.cachedHCR[1][2] = self.cachedHCR[1][2] - self.y_offsetCR
			self.cachedHCR_INV = np.linalg.inv(self.cachedHCR) 

			for idx, corner in enumerate(self.corners):
				self.corners[idx][1] = corner[1] - self.y_offsetCR
			self.source_R_corners = self.corners[0:4]
			self.source_C_corners = self.corners[4:8]
			
		# Find first point of R image in C image
		if self.x_coord_CR_limit is None:
			dmax = max(imageC.shape[:2])
			for pt in self.source_R_corners:
				if pt[0] < imageC.shape[1]:
					d = imageR.shape[1] - pt[0]
					if d < dmax:
						dmax = d
						self.x_coord_CR_limit = int(pt[0])
			mask_size =  int(imageC.shape[1] - self.x_coord_CR_limit)
			alpha_mask_CR = np.zeros((imageC.shape[0], mask_size, 3), np.float32)
			for x_coord in range(0, mask_size):
				alpha_mask_CR[0:alpha_mask_CR.shape[0], x_coord] = \
					float(x_coord)/float(mask_size)
			self.alpha_mask_CR = cv2.flip(alpha_mask_CR[::], 1)
			self.alpha_mask_CR2 = np.ones((imageC.shape[0], mask_size, 3), np.float32) - self.alpha_mask_CR

		# apply a perspective transform to stitch the images together
		result_CR = cv2.warpPerspective(imageR, self.cachedHCR, self.warp_CR_size)
		
		# Process super mode
		if self.super_mode:
			Ca = imageC[:, self.x_coord_CR_limit:]/255.
			Cb = result_CR[self.source_C_corners[0][1]: self.source_C_corners[2][1], 
						   self.x_coord_CR_limit      : imageC.shape[1]]/255. 
			Cr = (Ca*self.alpha_mask_CR + Cb*self.alpha_mask_CR2)*255
		
		# Overlay the center image to match
		result_CR[self.source_C_corners[0][1]: self.source_C_corners[2][1], 
				  0							 : self.source_C_corners[1][0]] = imageC
		
		# Process super mode
		if self.super_mode:
			result_CR[self.source_C_corners[0][1]: self.source_C_corners[2][1], 
				      self.x_coord_CR_limit : imageC.shape[1]] = Cr
		
		# Select ROI of first panoramic stitching 
		result_CR = result_CR[0:self.out_limitsCR[1] - self.y_offsetCR, 
							  0:self.out_limitsCR[3]]

		# Assign panoramic size
		self.pano_size_CR = (result_LC.shape[1], result_LC.shape[0])

		# Correct result perspective
		if self.cachedMCR is None:
			# Calculate rotation matrix from surface from original source image to 
			# projected four points surfaces
			src_points = np.array( corners, dtype=np.float32) 
			dst_points = np.array([[0                , 0               ], 
								  [result_LC.shape[1] , 0               ], 
								  [0                , result_LC.shape[0]],
								  [result_LC.shape[1] , result_LC.shape[0]]],dtype=np.float32)
			self.cachedMCR = cv2.getPerspectiveTransform(src_points, dst_points)
			self.cachedMCR_INV = np.linalg.inv(self.cachedMCR)

		result_CR = cv2.warpPerspective(result_CR, self.cachedMCR, self.pano_size_CR)

		self.pano_size_CR = (result_CR.shape[1], result_CR.shape[0])

		# ---------------------------------------------------------------------
		# if the cached homography matrix is None for pair L/CR, then we need to
		# apply key-point matching to construct it
		if self.cachedHLCR is None:

			save_file = True

			self.warp_size_LCR = (int(result_CR.shape[1] + imageC.shape[1]), 
					     		  int(result_CR.shape[0] + imageC.shape[0]))

			# detect key-points and extract
			(kpsAB, featuresAB) = self.detectAndDescribe(image = result_CR)
			(kpsC, featuresC)   = self.detectAndDescribe(image = imageL)
			
			# match features between the two images
			MLCR = self.matchKeypoints(kpsA = kpsAB, kpsB = kpsC,
									   featuresA = featuresAB, featuresB = featuresC, 
									   ratio = ratio, reprojThresh = reprojThresh)

			# if the match is None, then there aren't enough matched
			# key-points to create a panorama
			if MLCR is None:
				printlog(msg="Matched no possible for right + center + left stitching mode", 
					msg_type="ERROR")
				return result_CR
			else:
				printlog(msg="Right + center + left stitching matched successfully", 
					msg_type="INFO")

			# cache the homography matrix
			self.cachedHLCR = MLCR[1]
			
			for idx, pt in enumerate(self.corners):
				self.corners[idx] = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHLCR)
			
			self.corners.append(np.array((0, 0)))
			self.corners.append(np.array((imageL.shape[1], imageL.shape[0])))
			self.corners.append(np.array((0, imageL.shape[0])))
			self.corners.append(np.array((imageL.shape[1], 0)))

			self.out_limitsLCR, _, corners = get_extreme_contour_corners(self.corners)

			self.y_offsetLCR = self.out_limitsLCR[0]
			self.cachedHLCR[1][2] = self.cachedHLCR[1][2] - self.y_offsetLCR
			self.cachedHLCR_INV = np.linalg.inv(self.cachedHLCR)

			for idx, corner in enumerate(self.corners):
				self.corners[idx][1] = corner[1] - self.y_offsetLCR
			self.source_L_corners = self.corners[8:12]

		# Find first point of A image in B image
		if self.x_coord_LCR_limit is None:
			dmax = max(result_CR.shape[:2]) + imageL.shape[1]
			for pt in self.corners[0:8]:
				if pt[0] < imageL.shape[1]:
					d = imageL.shape[1] - pt[0]
					if d < dmax:
						dmax = d
						self.x_coord_LCR_limit = int(pt[0])
			mask_size =  int(imageL.shape[1] - self.x_coord_LCR_limit)
			alpha_mask_LCR = np.zeros((imageL.shape[0], mask_size, 3), np.float32)
			for x_coord in range(0, mask_size):
				alpha_mask_LCR[0:alpha_mask_LCR.shape[0], x_coord] = \
					float(x_coord)/float(mask_size)
			self.alpha_mask_LCR  = cv2.flip(alpha_mask_LCR[::], 1)
			self.alpha_mask_LCR2 = np.ones((imageL.shape[0], mask_size, 3), np.float32) - self.alpha_mask_LCR

		# apply a perspective transform to stitch the images together
		result_LCR = cv2.warpPerspective(src   = result_CR, 
										 M	   = self.cachedHLCR, 
										 dsize = self.warp_size_LCR)
		# Process with super mode
		if self.super_mode:
			Ca = imageL[:, self.x_coord_LCR_limit:]/255.
			Cb = result_LCR[
				self.source_L_corners[0][1]: self.source_L_corners[2][1], 
				self.x_coord_LCR_limit      : imageL.shape[1]]/255. 
			Cr = (Ca*self.alpha_mask_LCR + Cb*self.alpha_mask_LCR2)*255
		
		# Overlay the left image to match
		result_LCR[
			self.source_L_corners[0][1]: self.source_L_corners[1][1], 
			self.source_L_corners[0][0]: self.source_L_corners[1][0]] = imageL
		
		# Process with super mode
		if self.super_mode:
			result_LCR[
			self.source_L_corners[0][1]: self.source_L_corners[1][1], 
			self.x_coord_LCR_limit : imageL.shape[1]] = Cr
		
		# Select ROI of first panoramic stitching 
		result_LCR = result_LCR[0 : self.out_limitsLCR[1] - self.y_offsetLCR, 
								0 : self.out_limitsLCR[3]]

		# Correct result perspective
		if self.cachedMLCR is None:
			# Calculate rotation matrix from surface from original source image to 
			# projected four points surfaces
			src_points = np.array(corners, dtype=np.float32) 
			dst_points = np.array([[0                , 0               ], 
								[result_LCR.shape[1] , 0               ], 
								[0                , result_LCR.shape[0]],
								[result_LCR.shape[1] , result_LCR.shape[0]]],dtype=np.float32)
			self.cachedMLCR = cv2.getPerspectiveTransform(src_points, dst_points)
			self.cachedMLCR_INV = np.linalg.inv(self.cachedMLCR)
			for idx, pt in enumerate(self.corners):
				self.corners[idx] = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLCR)
			
			# Size of panoramic - (type) = (width, height)
			self.pano_size = (int(result_LCR.shape[1]), int(result_LCR.shape[0]))

			_, self.in_limitsLCR, _ = get_extreme_contour_corners(self.corners)

		printlog(msg="perspective transformation matrix calculated", 
			msg_type="INFO")
		printlog(msg="Stitching matcher calibrated successfully", 
			msg_type="INFO")

		# --------------------------------------------------------------------
		# Process with super mode
		result_LCR = cv2.warpPerspective(
			src = result_LCR, 
			M = self.cachedMLCR, 
			dsize = self.pano_size)
		
		# Final warp to get final panoramic stitching		
		result_LCR = result_LCR[self.in_limitsLCR[0]:self.in_limitsLCR[1], :]

		self.pano_size_LCR = (result_LCR.shape[1], result_LCR.shape[0])

		# ---------------------------------------------------------------------
		pt = [int(imageL.shape[1]*0.5), int(imageL.shape[0]*0.5)]
		pt[1] += self.source_L_corners[0][1]
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLCR)
		pt[1] -= self.in_limitsLCR[0]
		self.LCR_Lc = pt

		pt = [int(imageC.shape[1]*0.5), int(imageC.shape[0]*0.5)]
		pt[1] += self.source_C_corners[0][1]
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHLCR)
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLCR)
		pt[1] -= self.in_limitsLCR[0]
		self.LCR_Cc = pt

		pt = [int(imageR.shape[1]*0.5), int(imageR.shape[0]*0.5)]
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHCR)
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHLCR)
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLCR)
		pt[1] -= self.in_limitsLCR[0]
		self.LCR_Rc = pt

		# ---------------------------------------------------------------------
		pt = [int(imageL.shape[1]*0.5), int(imageL.shape[0]*0.5)]
		pt[1] += self.source_L_LC_corners[0][1]
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLC)
		self.LC_Lc  = pt

		pt = [int(imageC.shape[1]*0.5), int(imageC.shape[0]*0.5)]
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHLC)
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLC)		
		self.LC_Cc  = pt

		# ---------------------------------------------------------------------
		pt = [int(imageR.shape[1]*0.5), int(imageR.shape[0]*0.5)]
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHCR)
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMCR)
		self.CR_Rc  = pt

		pt = [int(imageC.shape[1]*0.5), int(imageC.shape[0]*0.5)]
		pt[1] += self.source_C_corners[0][1]
		pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMCR)
		self.CR_Cc  = pt

		# ---------------------------------------------------------------------
		# Save stitcher configuration
		if save_file: self.save_stitcher(abs_path = self.path_config_file)

		# ---------------------------------------------------------------------
		return result_LCR

	def stitch(self, images, ratio=0.75, reprojThresh=4.0, pan=0):

		# ---------------------------------------------------------------------
		# unpack the images
		# Left , Center, Right -> Streaming
		(imageL, imageC, imageR) = images

		# ---------------------------------------------------------------------
		#  CENTER (C) = (C)
		if imageL is None and imageR is None:
			self.stitch_mode = [0, 1, 0]
			return imageC

		# ---------------------------------------------------------------------
		# LEFT (L) + CENTER (C) = (RC)
		if imageR is None and imageC is not None and imageL is not None:

			# If configuration do process
			if not self.cachedHLC is None:

				# Get perspective transform to mathch Center image in Left image
				result_LC = cv2.warpPerspective(src = imageC, 
												M = self.cachedHLC, 
												dsize = self.warp_LC_size,
												flags = self.warp_flags)

				# Process super mode
				if self.super_mode:
					Ca = imageL[:, self.x_coord_LC_limit:]/255.
					Cb = result_LC[
						self.source_L_LC_corners[0][1]: self.source_L_LC_corners[2][1], 
						self.x_coord_LC_limit  	     : imageL.shape[1]]/255. 

					if (self.alpha_mask_LC.shape[1] != Ca.shape[1] or 
						self.alpha_mask_LC.shape[0] != Ca.shape[0] ):
						new_size = (int(Ca.shape[1]),
									int(Ca.shape[0]))
						self.alpha_mask_LC = cv2.resize(self.alpha_mask_LC, new_size)
						self.alpha_mask_LC2 = cv2.resize(self.alpha_mask_LC2, new_size)

					Cr = (Ca*self.alpha_mask_LC + Cb*self.alpha_mask_LC2)*255

				# Overlay the center image to match
				result_LC[self.source_L_LC_corners[0][1]: self.source_L_LC_corners[2][1], 
						0					            : self.source_L_LC_corners[1][0]] = imageL

				# Process super mode
				if self.super_mode:
					result_LC[self.source_L_LC_corners[0][1]: self.source_L_LC_corners[2][1], 
							self.x_coord_LC_limit           : imageL.shape[1]] = Cr

				# Select ROI of first panoramic stitching 
				result_LC = result_LC[
					0:self.out_limitsLC[1] - self.y_offsetLC, 
					0:self.out_limitsLC[3]]

				# Adjust perpective
				result_LC = cv2.warpPerspective(src = result_LC, 
												M = self.cachedMLC, 
												dsize = self.pano_size_LC,
												flags = self.warp_flags)

				# Return result
				self.stitch_mode = [1, 1, 0]
				return result_LC

			else:
				np.concatenate((imageL, imageC), axis = 1)

		if not self.cachedHCR is None and not self.cachedHLCR is None:
			# ---------------------------------------------------------------------
			# CENTER (C) + RIGHT (R) = (CR)

			# apply a perspective transform to stitch the images together			
			result_CR = cv2.warpPerspective(src = imageR, 
											M = self.cachedHCR, 
											dsize = self.warp_CR_size,
											flags = self.warp_flags)

			# Process super mode
			if self.super_mode:
				Ca = imageC[:, self.x_coord_CR_limit:]/255.
				Cb = result_CR[self.source_C_corners[0][1]: self.source_C_corners[2][1], 
							self.x_coord_CR_limit      : imageC.shape[1]]/255. 
				Cr = (Ca*self.alpha_mask_CR + Cb*self.alpha_mask_CR2)*255
			
			# Overlay the center image to match
			result_CR[
				self.source_C_corners[0][1]: self.source_C_corners[2][1], 
				0							 : self.source_C_corners[1][0]] = imageC
			
			# Process super mode
			if self.super_mode:
				result_CR[self.source_C_corners[0][1]: self.source_C_corners[2][1], 
						self.x_coord_CR_limit : imageC.shape[1]] = Cr
			
			# Select ROI of first panoramic stitching 
			result_CR = result_CR[
				0:self.out_limitsCR[1] - self.y_offsetCR, 
				0:self.out_limitsCR[3]]

			if imageL is None:

				result_CR = cv2.warpPerspective(
					src = result_CR, 
					M = self.cachedMCR, 
					dsize = self.pano_size_CR,
					flags = self.warp_flags)
				self.stitch_mode = [0, 1, 1]
				return result_CR

			# -----------------------------------------------------------------
			# LEFT (L) + CENTER&RIGHT (C&R)= (LC&R)

			# apply a perspective transform to stitch the images together
			result_LCR = cv2.warpPerspective(
				src   = result_CR, 
				M	  = self.cachedHLCR, 
				dsize = self.warp_size_LCR,
				flags = self.warp_flags)

			# Process with super mode
			if self.super_mode:
				Ca = imageL[:, self.x_coord_LCR_limit:]/255.
				Cb = result_LCR[self.source_L_corners[0][1]: self.source_L_corners[2][1], 
								self.x_coord_LCR_limit      : imageL.shape[1]]/255. 
				Cr = (Ca*self.alpha_mask_LCR + Cb*self.alpha_mask_LCR2)*255
			
			# Overlay the left image to match
			result_LCR[
				self.source_L_corners[0][1]: self.source_L_corners[1][1], 
				self.source_L_corners[0][0]: self.source_L_corners[1][0]] = imageL
			
			# Process with super mode
			if self.super_mode:
				result_LCR[self.source_L_corners[0][1]: self.source_L_corners[1][1], 
						self.x_coord_LCR_limit : imageL.shape[1]] = Cr
			
			# Select ROI of first panoramic stitching 
			result_LCR = result_LCR[0 : self.out_limitsLCR[1] - self.y_offsetLCR, 
									0 : self.out_limitsLCR[3]]

			# --------------------------------------------------------------------
			# Process with super mode
			result_LCR = cv2.warpPerspective(
				src = result_LCR, 
				M = self.cachedMLCR, 
				dsize = self.pano_size,
				flags = self.warp_flags)
			
			# Final warp to get final panoramic stitching		
			result_LCR = result_LCR[self.in_limitsLCR[0]:self.in_limitsLCR[1], :]

			# Return result
			self.stitch_mode = [1, 1, 1]
			return result_LCR

		else:
			self.stitch_mode = [0, 0, 0]
			np.concatenate((imageL, imageC, imageR), axis = 1)

	def save_stitcher(self, abs_path="", quite=False):
		""" Functions saves the stitcher configuration file in a given absolute 
			path, and at root folder if not specified
		Args:
			abs_path: `string` absolute path to save stitcher configuration file
		Returns:
		"""

		if abs_path == "":
			abs_path = "./default_config.npz"

		try:
			np.savez(os.path.join(abs_path), 
					pano_size 		= self.pano_size,  
					# super_mode = self.super_mode,
					pano_size_LCR	= self.pano_size_LCR,
					stitch_mode		= self.stitch_mode,
					warp_flags 		= self.warp_flags,
					cachedHCR 		= self.cachedHCR,
					cachedHCR_INV 	= self.cachedHCR_INV,
					Window_size 	= self.Window_size,
					y_offsetCR 		= self.y_offsetCR,
					warp_CR_size 	= self.warp_CR_size, 
					out_limitsCR 	= self.out_limitsCR, 
					in_limitsCR		= self.in_limitsCR,
					vir_pan_val 	= self.vir_pan_val,
					x_coord_CR_limit = self.x_coord_CR_limit,
					alpha_mask_CR 	= self.alpha_mask_CR,
					alpha_mask_CR2 	= self.alpha_mask_CR2, 
					cachedMCR 		= self.cachedMCR,
					cachedMCR_INV 	= self.cachedMCR_INV,
					pano_size_CR 	= self.pano_size_CR, 
					cachedHLC 		= self.cachedHLC,
					cachedHLC_INV 	= self.cachedHLC_INV,
					warp_LC_size 	= self.warp_LC_size,
					x_coord_LC_limit = self.x_coord_LC_limit, 
					alpha_mask_LC 	= self.alpha_mask_LC,
					alpha_mask_LC2 	= self.alpha_mask_LC2,
					source_C_LC_corners = self.source_C_LC_corners,
					source_L_LC_corners = self.source_L_LC_corners, 
					out_limitsLC 	= self.out_limitsLC, 
					in_limitsLC 	= self.in_limitsLC,
					y_offsetLC 		= self.y_offsetLC,
					cachedMLC 		= self.cachedMLC,
					cachedMLC_INV 	= self.cachedMLC_INV,
					pano_size_LC 	= self.pano_size_LC, 
					cachedHLCR 		= self.cachedHLCR, 
					cachedHLCR_INV 	= self.cachedHLCR_INV, 
					cachedMLCR 		= self.cachedMLCR,  
					cachedMLCR_INV 	= self.cachedMLCR_INV,  
					y_offsetLCR 	= self.y_offsetLCR,
					warp_size_LCR 	= self.warp_size_LCR,
					out_limitsLCR 	= self.out_limitsLCR, 	  
					in_limitsLCR 	= self.in_limitsLCR,
					source_R_corners 	= self.source_R_corners, 
					source_L_corners 	= self.source_L_corners,
					source_C_corners 	= self.source_C_corners, 
					x_coord_LCR_limit 	= self.x_coord_LCR_limit,
					alpha_mask_LCR 		= self.alpha_mask_LCR,	
					alpha_mask_LCR2 	= self.alpha_mask_LCR2,
					corners = self.corners,	
					PP_CntL = self.PP_CntL,
					PP_CntC = self.PP_CntC,
					PP_CntR = self.PP_CntR,
					LC_CtnL = self.LC_CtnL,
					LC_CtnC = self.LC_CtnC,
					CR_CtnC = self.CR_CtnC,
					CR_CtnR = self.CR_CtnR,
					LCR_CtnL = self.LCR_CtnL,
					LCR_CtnC = self.LCR_CtnC,
					LCR_CtnR = self.LCR_CtnR,
					LC_CtnL_Mc = self.LC_CtnL_Mc,
					LC_CtnC_Mc = self.LC_CtnC_Mc, 
					CR_CtnC_Mc = self.CR_CtnC_Mc, 
					CR_CtnR_Mc = self.CR_CtnR_Mc, 
					LCR_CtnL_Mc = self.LCR_CtnL_Mc,
					LCR_CtnC_Mc = self.LCR_CtnC_Mc,
					LCR_CtnR_Mc = self.LCR_CtnR_Mc,
					LCR_Lc = self.LCR_Lc,
					LCR_Cc = self.LCR_Cc,
					LCR_Rc = self.LCR_Rc,
					LC_Lc = self.LC_Lc,
					LC_Cc = self.LC_Cc,
					CR_Rc = self.CR_Rc,
					CR_Cc = self.CR_Cc,
					version  = 1.4)

			# Print some information 
			if not quite:
				printlog(msg="Stitcher configuration saved: {}".format(
					os.path.join(abs_path)), msg_type="INFO")

		except Exception as e:
			printlog(msg="something bad occurred saving stitcher configuration file: {}".format(e), 
				msg_type="ERROR")
			return 1

		return 0		

	def load_stitcher(self, source_path, quiet=False):
		""" Functions loads the stitcher configuration file in a given absolute 
			path, and at root folder if not specified
		Args:
			abs_path: `string` absolute path to load stitcher configuration file
		Returns:
		"""

		try:
			config = np.load(source_path)

			self.pano_size = tuple(config["pano_size"])
			self.stitch_mode = config["stitch_mode"]
			# self.super_mode = config["super_mode"]
			self.cachedHCR = config["cachedHCR"]
			self.vir_pan_val = config["vir_pan_val"]
			self.cachedHCR_INV = config["cachedHCR_INV"]
			self.Window_size = tuple(config["Window_size"])
			self.warp_flags = int(config["warp_flags"])
			self.y_offsetCR = config["y_offsetCR"]
			self.warp_CR_size = tuple(config["warp_CR_size"])
			self.out_limitsCR = config["out_limitsCR"]
			self.in_limitsCR = config["in_limitsCR"]
			self.x_coord_CR_limit = config["x_coord_CR_limit"]
			self.alpha_mask_CR = config["alpha_mask_CR"]
			self.alpha_mask_CR2 = config["alpha_mask_CR2"]
			self.cachedMCR = config["cachedMCR"]
			self.cachedMCR_INV = config["cachedMCR_INV"]
			self.pano_size_CR = tuple(config["pano_size_CR"])
			self.cachedHLC = config["cachedHLC"]
			self.cachedHLC_INV = config["cachedHLC_INV"]
			self.warp_LC_size = tuple(config["warp_LC_size"])
			self.x_coord_LC_limit = config["x_coord_LC_limit"]
			self.alpha_mask_LC = config["alpha_mask_LC"]
			self.alpha_mask_LC2 = config["alpha_mask_LC2"]
			self.source_C_LC_corners = config["source_C_LC_corners"]
			self.source_L_LC_corners = config["source_L_LC_corners"]
			self.out_limitsLC = config["out_limitsLC"]
			self.in_limitsLC = config["in_limitsLC"]
			self.y_offsetLC = config["y_offsetLC"]
			self.cachedMLC = config["cachedMLC"]
			self.cachedMLC_INV = config["cachedMLC_INV"]
			self.pano_size_LC = tuple(config["pano_size_LC"])
			self.cachedHLCR = config["cachedHLCR"]
			self.cachedMLCR = config["cachedMLCR"]
			self.cachedHLCR_INV = config["cachedHLCR_INV"]
			self.cachedMLCR_INV = config["cachedMLCR_INV"]
			self.y_offsetLCR = config["y_offsetLCR"]
			self.warp_size_LCR = tuple(config["warp_size_LCR"])
			self.out_limitsLCR = config["out_limitsLCR"] 
			self.in_limitsLCR = config["in_limitsLCR"]
			self.source_R_corners = config["source_R_corners"]
			self.source_L_corners = config["source_L_corners"]
			self.source_C_corners = config["source_C_corners"]
			self.x_coord_LCR_limit = config["x_coord_LCR_limit"]
			self.alpha_mask_LCR = config["alpha_mask_LCR"]
			self.alpha_mask_LCR2 = config["alpha_mask_LCR2"]
			self.corners = config["corners"]
			self.PP_CntC = None if not config["PP_CntC"].any() else config["PP_CntC"]
			self.PP_CntR = None if not config["PP_CntR"].any() else config["PP_CntR"]
			self.LC_CtnL = None if not config["LC_CtnL"].any() else config["LC_CtnL"]
			self.LC_CtnC = None if not config["LC_CtnC"].any() else config["LC_CtnC"]
			self.CR_CtnC = None if not config["CR_CtnC"].any() else config["CR_CtnC"]
			self.CR_CtnR = None if not config["CR_CtnR"].any() else config["CR_CtnR"]
			self.LCR_CtnL = None if not config["LCR_CtnL"].any() else config["LCR_CtnL"]
			self.LCR_CtnC = None if not config["LCR_CtnC"].any() else config["LCR_CtnC"]
			self.LCR_CtnR = None if not config["LCR_CtnR"].any() else config["LCR_CtnR"]
			self.LC_CtnL_Mc = config["LC_CtnL_Mc"]
			self.LC_CtnC_Mc = config["LC_CtnC_Mc"]
			self.CR_CtnC_Mc = config["CR_CtnC_Mc"]
			self.CR_CtnR_Mc = config["CR_CtnR_Mc"]
			self.LCR_CtnL_Mc = config["LCR_CtnL_Mc"]
			self.LCR_CtnC_Mc = config["LCR_CtnC_Mc"]
			self.LCR_CtnR_Mc = config["LCR_CtnR_Mc"]
			self.LCR_Lc = config["LCR_Lc"]
			self.LCR_Cc = config["LCR_Cc"]
			self.LCR_Rc = config["LCR_Rc"]
			self.LC_Lc = config["LC_Lc"]
			self.LC_Cc = config["LC_Cc"]
			self.CR_Rc = config["CR_Rc"]
			self.CR_Cc = config["CR_Cc"]
			self.pano_size_LCR = tuple(config["pano_size_LCR"])
			self.version = config["version"]
	
			if not quiet:
				printlog(msg="stitcher configuration loaded", 
					msg_type="INFO")

		except Exception as e:

			self.pano_size = None
			self.stitch_mode = [0, 0, 0]
			# self.super_mode = config["super_mode"]
			self.cachedHCR = None
			self.cachedHCR_INV = None
			self.Window_size = (640, 360)
			self.warp_flags = int(cv2.INTER_NEAREST)
			self.vir_pan_val = 0
			self.y_offsetCR = None
			self.warp_CR_size = None
			self.out_limitsCR = None
			self.in_limitsCR = None
			self.x_coord_CR_limit = None
			self.alpha_mask_CR = None
			self.alpha_mask_CR2 = None
			self.cachedMCR = None
			self.cachedMCR_INV = None
			self.pano_size_CR = None
			self.pano_size_LCR = None
			self.cachedHLC = None
			self.cachedHLC_INV = None
			self.warp_LC_size = None
			self.x_coord_LC_limit = None
			self.alpha_mask_LC = None
			self.alpha_mask_LC2 = None
			self.source_C_LC_corners = []
			self.source_L_LC_corners = []
			self.out_limitsLC = None
			self.in_limitsLC = None
			self.y_offsetLC = None
			self.cachedMLC = None
			self.cachedMLC_INV = None
			self.pano_size_LC = None
			self.cachedHLCR = None
			self.cachedMLCR = None
			self.cachedHLCR_INV = None
			self.cachedMLCR_INV =  None
			self.y_offsetLCR = None
			self.warp_size_LCR = None
			self.out_limitsLCR = None
			self.in_limitsLCR = None
			self.source_R_corners = []
			self.source_L_corners = []
			self.source_C_corners = []
			self.x_coord_LCR_limit = None
			self.alpha_mask_LCR = None
			self.alpha_mask_LCR2 = None
			self.corners = []
			self.PP_CntL = None
			self.PP_CntC = None
			self.PP_CntR = None
			self.LC_CtnL = None
			self.LC_CtnC = None
			self.CR_CtnC = None
			self.CR_CtnR = None
			self.LCR_CtnL = None
			self.LCR_CtnC = None
			self.LCR_CtnR = None
			self.LC_CtnL_Mc = None
			self.LC_CtnC_Mc = None
			self.CR_CtnC_Mc = None
			self.CR_CtnR_Mc = None
			self.LCR_CtnL_Mc = None
			self.LCR_CtnC_Mc = None
			self.LCR_CtnR_Mc = None
			self.LCR_Lc = None
			self.LCR_Cc = None
			self.LCR_Rc = None
			self.LC_Lc = None
			self.LC_Cc = None
			self.CR_Rc = None
			self.CR_Cc = None
			self.version = "X.X?"

			printlog(msg="loading stitcher: {}".format(e), 
				msg_type="ERROR")
			printlog(msg="parameters established as None and empty arrays, please check configuration file", 
				msg_type="WARN")

	def detectAndDescribe(self, image):

		""" description
		Args:
			image: `cv2.math` input image to find descriptors and features
		Returns:
			kps: `numpy.ndarray` collection of key-points. Key-points for which a
				descriptor cannot be computed are removed. Sometimes new 
				key-points can be added, for example: SIFT duplicates key-point 
				with several dominant orientations (for each orientation).
			features: `numpy.ndarray`  Computed descriptors. In the second 
				variant of the method descriptors[i] are descriptors 
				computed for a key-points[i]. Row j is the key-points 
				(or key-points[i]) is the descriptor for key-point j-th 
				key-point.
		"""

		# convert the image to gray scale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# check to see if we are using OpenCV 3.X
		if self.isv3:

			# detect and extract features from the image
			descriptor = cv2.xfeatures2d.SIFT_create()
			(kps, features) = descriptor.detectAndCompute(image, None)

		# otherwise, we are using OpenCV 2.4.X
		else:

			# detect key-points in the image
			detector = cv2.FeatureDetector_create("SIFT")
			kps = detector.detect(gray)

			# extract features from the image
			extractor = cv2.DescriptorExtractor_create("SIFT")
			(kps, features) = extractor.compute(gray, kps)

		# convert the key-points from Key-point objects to NumPy
		# arrays
		kps = np.float32([kp.pt for kp in kps])

		# return a tuple of key-points and features
		return (kps, features)

	def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh):
		
		""" Match correlated points between lists featuresA and featuresB
		Args:
			kpsA: `numpy.ndarray` Coordinates of the points in the original plane
			kpsB: `numpy.ndarray` Coordinates of the points in the target plane
			featuresA: `numpy.ndarray` list of features in original plane
			featuresB: `numpy.ndarray` list of features in target plane
			ratio: 'float' aspect ratio for panoramic result (Keep it in 0.75)
			reprojThresh: 'float' correlation value for desired matched points 
		Returns:
			matches: `list` description
			H: `numpy.ndarray` perspective transformation between the source 
				and the destination planes
			status: `numpy.ndarray` list of correlation state for each feature paired
		"""

		# compute the raw matches and initialize the list of actual
		# matches
		matcher = cv2.DescriptorMatcher_create("BruteForce")
		rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
		matches = []

		# loop over the raw matches
		for m in rawMatches:

			# ensure the distance is within a certain ratio of each
			# other (i.e. Lowe's ratio test)
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				matches.append((m[0].trainIdx, m[0].queryIdx))

		# computing a homography requires at least 4 matches
		if len(matches) > 4:

			# construct the two sets of points
			ptsA = np.float32([kpsA[i] for (_, i) in matches])
			ptsB = np.float32([kpsB[i] for (i, _) in matches])

			# compute the homography between the two sets of points
			(H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
				reprojThresh)

			# return the matches along with the homography matrix
			# and status of each matched point
			return (matches, H, status)

		# otherwise, no homography could be computed
		return None

	def draw_stitched_projections(self, img_src):

		""" Draws surface projections in stitching space
		Args:
			img_src: `cv2.math` original image to draw surface projections
		Returns:
			img_src: `cv2.math` original image with surface projections drawn
		"""

		y_offset = 20; x_offset = 3
		thickness = 2

		# ---------------------------------------------------------------------
		# For stitching mode Left Camera + Center Camera + Right Camera = "LCR"
		if (self.stitch_mode[0] == 1 and self.stitch_mode[1] == 1 and self.stitch_mode[2] == 1): 
			if self.LCR_CtnL is not None: 
				cv2.drawContours(img_src, [self.LCR_CtnL], -1, (255, 255, 0), thickness)
			pts_ll = np.array([[self.LCR_Lc[0], 0], [self.LCR_Lc[0]+10, 20], [self.LCR_Lc[0]+20, 0]], np.int32)
			cv2.putText(img_src, "LL", (self.LCR_Lc[0] + x_offset, 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5, lineType=cv2.LINE_AA)
			cv2.putText(img_src, "LL", (self.LCR_Lc[0] + x_offset, 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, lineType=cv2.LINE_AA)
			cv2.drawContours(img_src, [pts_ll], -1, (0, 0, 255), -1)
			
			if self.LCR_CtnC is not None: 
				cv2.drawContours(img_src, [self.LCR_CtnC], -1, (0, 255, 0), thickness)
			pts_c = np.array([[self.LCR_Cc[0], 0], [self.LCR_Cc[0]+10, 20], [self.LCR_Cc[0]+20, 0]], np.int32)
			cv2.putText(img_src, "C", (self.LCR_Cc[0]+5, 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5, lineType=cv2.LINE_AA)
			cv2.putText(img_src, "C", (self.LCR_Cc[0]+5, 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, lineType=cv2.LINE_AA)
			cv2.drawContours(img_src, [pts_c], -1, (0, 0, 255), -1)
			
			if self.LCR_CtnR is not None: 
				cv2.drawContours(img_src, [self.LCR_CtnR], -1, (0, 255, 255), thickness)
			pts_rr = np.array([[self.LCR_Rc[0], 0], [self.LCR_Rc[0]+10, 20], [self.LCR_Rc[0]+20, 0]], np.int32)
			cv2.putText(img_src, "RR", (self.LCR_Rc[0], 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5, lineType=cv2.LINE_AA)
			cv2.putText(img_src, "RR", (self.LCR_Rc[0], 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, lineType=cv2.LINE_AA)
			cv2.drawContours(img_src, [pts_rr], -1, (0, 0, 255), -1)

		# ---------------------------------------------------------------------
		# For stitching mode Left Camera + Center Camera = "LC"
		elif (self.stitch_mode[0] == 1 and self.stitch_mode[1] == 1 and self.stitch_mode[2] == 0): 
			if self.LC_CtnL is not None: 
				for pt in self.LC_CtnL: cv2.circle(img_src, tuple(pt), 1, (255, 255, 0), -1)
			pts_rr = np.array([[self.LC_Lc[0], 0], [self.LC_Lc[0]+10, 20], [self.LC_Lc[0]+20, 0]], np.int32)
			cv2.putText(img_src, "LL", (self.LC_Lc[0], 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5, lineType=cv2.LINE_AA)
			cv2.putText(img_src, "LL", (self.LC_Lc[0], 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, lineType=cv2.LINE_AA)
			cv2.drawContours(img_src, [pts_rr], -1, (0, 0, 255), -1)
			if self.LC_CtnC is not None: 
				for pt in self.LC_CtnC: cv2.circle(img_src, tuple(pt), 1, (0, 255, 0), -1)
			pts_rr = np.array([[self.LC_Cc[0], 0], [self.LC_Cc[0]+10, 20], [self.LC_Cc[0]+20, 0]], np.int32)
			cv2.putText(img_src, "C", (self.LC_Cc[0] + 5, 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5, lineType=cv2.LINE_AA)
			cv2.putText(img_src, "C", (self.LC_Cc[0] + 5, 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, lineType=cv2.LINE_AA)
			cv2.drawContours(img_src, [pts_rr], -1, (0, 0, 255), -1)

		# ---------------------------------------------------------------------
		# For stitching mode Center Camera + Right Camera = "CR"
		elif (self.stitch_mode[0] == 0 and self.stitch_mode[1] == 1 and self.stitch_mode[2] == 1): 
			if self.CR_CtnC is not None: 
				for pt in self.CR_CtnC: cv2.circle(img_src, tuple(pt), 1, (0, 255, 0), -1)
			pts_rr = np.array([[self.CR_Cc[0], 0], [self.CR_Cc[0]+10, 20], [self.CR_Cc[0]+20, 0]], np.int32)
			cv2.putText(img_src, "C", (self.CR_Cc[0] + 5, 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5, lineType=cv2.LINE_AA)
			cv2.putText(img_src, "C", (self.CR_Cc[0] + 5, 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, lineType=cv2.LINE_AA)
			cv2.drawContours(img_src, [pts_rr], -1, (0, 0, 255), -1)
			if self.CR_CtnR is not None: 
				for pt in self.CR_CtnR: cv2.circle(img_src, tuple(pt), 1, (0, 0, 255), -1)
			pts_rr = np.array([[self.CR_Rc[0], 0], [self.CR_Rc[0]+10, 20], [self.CR_Rc[0]+20, 0]], np.int32)
			cv2.putText(img_src, "RR", (self.CR_Rc[0], 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5, lineType=cv2.LINE_AA)
			cv2.putText(img_src, "RR", (self.CR_Rc[0], 20 + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, lineType=cv2.LINE_AA)
			cv2.drawContours(img_src, [pts_rr], -1, (0, 0, 255), -1)

		return img_src

	def get_coord_source(self, coord, ):
		
		""" Calculates coordinate's (coord) source and respective coordinate in
			source
		Args:
			coord: `tuple` coordinate in sticking image to calculate camera
			source and equivalent coordinate there 
		Returns:
			Origin: `string` camera label source: LL = Left, C = Center, RR = Right
			Coord_src: `tuple` equivalent coordinate in source
		"""

		# ---------------------------------------------------------------------
		Origin = None
		Coord_src = None
		Coord_stitch = None

		# ---------------------------------------------------------------------
		# For stitching mode Left Camera + Center Camera + Right Camera = "LCR"
		if (self.stitch_mode[0] == 1 and self.stitch_mode[1] == 1 and self.stitch_mode[2] == 1): 
			
			if self.vir_pan_val > 100 or self.vir_pan_val < -100:
				coord[0] = int(coord[0]*self.pano_size_LCR[0])
				r = float(self.Window_size[0])/float(self.pano_size_LCR[0])
				dl = (self.Window_size[1] - r*self.pano_size_LCR[1])*0.5
				coord[1] = coord[1]*self.Window_size[1] 
				if coord[1] <= dl or  coord[1] > self.Window_size[1] - dl:
					printlog(msg="LCR - click out of boundaries", 
						msg_type="WARN")
					return None, None, None
				coord[1] -= dl
				coord[1] = int(self.pano_size_LCR[1]*float(coord[1])/float(r*self.pano_size_LCR[1]))				
				coord = [int(coord[0]), int(coord[1])]
				Coord_stitch = coord

			else:
				x_offset = 0
				if self.vir_pan_val <= 0: 
					x_offset = int(self.LCR_Cc[0] - self.Window_size[0]/2 + self.vir_pan_val*(self.LCR_Cc[0] - self.Window_size[0]/2)/100.)
				if self.vir_pan_val > 0: 
					x_offset =  int(self.LCR_Cc[0] - self.Window_size[0]/2) +  int((self.vir_pan_val*(self.pano_size_LCR[0] - int(self.LCR_Cc[0] + self.Window_size[0]/2)))/100.) 
				coord[0]= (coord[0]*self.Window_size[0]) + x_offset
				coord[1]= coord[1]*self.pano_size_LCR[1]
				coord = [int(coord[0]), int(coord[1])]
				Coord_stitch = coord

			# Look for origin, checking if coordinate is inside in any contour

			if self.LCR_CtnL is not None: # If there's left camera calibration
				if cv2.pointPolygonTest(np.array(self.LCR_CtnL), tuple(coord), True) >= 0 : 
					Origin = "LL"
					if (self.cachedMLCR_INV is not None):
						Coord_src = np.array(Coord_stitch).copy()
						Coord_src[1] += self.in_limitsLCR[0]
						Coord_src = get_projection_point_dst(coords_src = (Coord_src[0], Coord_src[1], 1), M = self.cachedMLCR_INV)
						Coord_src[1] -= self.source_L_corners[0][1]
					if self.cachedMLCR_INV is None: 
						printlog(msg="LCR - Variable cachedMLCR_INV no defined", 
							msg_type="WARN")
					Coord_src = [int(Coord_src[0]), int(Coord_src[1])]
					return Origin, Coord_src, Coord_stitch

			if self.LCR_CtnC is not None: # If there's center camera calibration
				if cv2.pointPolygonTest(np.array(self.LCR_CtnC), tuple(coord), True) >= 0 : 
					Origin = "C"
					if (self.cachedMLCR_INV is not None) or (self.cachedHLCR_INV is not None):
						Coord_src = np.array(Coord_stitch).copy()
						Coord_src[1] += self.in_limitsLCR[0]
						Coord_src = get_projection_point_dst(coords_src = (Coord_src[0], Coord_src[1], 1), M = self.cachedMLCR_INV)
						Coord_src = get_projection_point_dst(coords_src = (Coord_src[0], Coord_src[1], 1), M = self.cachedHLCR_INV)
						Coord_src[1] -= self.source_C_corners[0][1]
					if self.cachedMLCR_INV is None: 
						printlog(msg="LCR - Variable cachedMLCR_INV no defined", 
							msg_type="WARN")
					if self.cachedHLCR_INV is None: 
						printlog(msg="LCR - Variable cachedHLCR_INV no defined", 
							msg_type="WARN")
					Coord_src = [int(Coord_src[0]), int(Coord_src[1])]
					return Origin, Coord_src, Coord_stitch

			if self.LCR_CtnR is not None: # If there's right camera calibration
				if cv2.pointPolygonTest(np.array(self.LCR_CtnR), tuple(coord), True) >= 0 : 
					Origin = "RR"
					if (self.cachedMLCR_INV is not None) or (self.cachedHLCR_INV is not None) or (self.cachedHCR_INV is not None):
						Coord_src = np.array(Coord_stitch).copy()
						Coord_src[1] += self.in_limitsLCR[0]
						Coord_src = get_projection_point_dst(coords_src = (Coord_src[0], Coord_src[1], 1), M = self.cachedMLCR_INV)
						Coord_src = get_projection_point_dst(coords_src = (Coord_src[0], Coord_src[1], 1), M = self.cachedHLCR_INV)
						Coord_src = get_projection_point_dst(coords_src = (Coord_src[0], Coord_src[1], 1), M = self.cachedHCR_INV)
					if self.cachedMLCR_INV is None: 
						printlog(msg="LCR - Variable cachedMLCR_INV no defined", 
							msg_type="WARN")
					if self.cachedHLCR_INV is None: 
						printlog(msg="LCR - Variable cachedHLCR_INV no defined", 
							msg_type="WARN")
					if self.cachedHCR_INV  is None: 
						printlog(msg="LCR - Variable cachedHCR_INV no defined", 
							msg_type="WARN")
					Coord_src = [int(Coord_src[0]), int(Coord_src[1])]
					return Origin, Coord_src, Coord_stitch

			return None, None, Coord_stitch

		# ---------------------------------------------------------------------
		# For stitching mode Left Camera + Center Camera = "LC"
		elif (self.stitch_mode[0] == 1 and self.stitch_mode[1] == 1 and self.stitch_mode[2] == 0): 

			if self.vir_pan_val > 0 or self.vir_pan_val < -100:
				printlog(msg="Virtual pan exceeds ranges for LC mode", 
					msg_type="WARN")
				return None, None, None
			x_offset = int((100 + self.vir_pan_val)*(self.pano_size_LC[0] - self.Window_size[0])/100.)    
			coord[0]= (coord[0]*self.Window_size[0]) + x_offset
			coord[1]= coord[1]*self.pano_size_LC[1]
			coord = [int(coord[0]), int(coord[1])]
			Coord_stitch = coord

			# Look for origin, checking if coordinate is inside in any contour

			if self.LC_CtnL is not None: # If there's left camera calibration
				if cv2.pointPolygonTest(np.array(self.LC_CtnL), tuple(coord), True) >= 0 : 
					Origin = "LL"
					if self.cachedMLC_INV is not None:
						Coord_src = get_projection_point_dst(coords_src = (coord[0], coord[1], 1), M = self.cachedMLC_INV)
						Coord_src[1] -= self.source_L_LC_corners[0][1]
					if self.cachedMLC_INV is None: 
						printlog(msg="Variable cachedMLC_INV no defined", 
							msg_type="WARN")
					Coord_src = [int(Coord_src[0]), int(Coord_src[1])]
					return Origin, Coord_src, Coord_stitch

			if self.LC_CtnC is not None: # If there's center camera calibration
				if cv2.pointPolygonTest(np.array(self.LC_CtnC), tuple(coord), True) >= 0 : 
					Origin = "C"
					if (self.cachedMLC_INV is not None) or (self.cachedHLC_INV is not None):
						Coord_src = get_projection_point_dst(coords_src = (coord[0], coord[1], 1), M = self.cachedMLC_INV)
						Coord_src = get_projection_point_dst(coords_src = (Coord_src[0], Coord_src[1], 1),  M = self.cachedHLC_INV)
					if self.cachedHLC_INV is None: 
						printlog(msg="Variable cachedHLC_INV no defined", 
							msg_type="WARN")
					if self.cachedMLC_INV is None: 
						printlog(msg="Variable cachedMLC_INV no defined", 
							msg_type="WARN")
					Coord_src = [int(Coord_src[0]), int(Coord_src[1])]
					return Origin, Coord_src, Coord_stitch

			return None, None, Coord_stitch

		# ---------------------------------------------------------------------
		# For stitching mode Center Camera + Right Camera = "CR"
		elif (self.stitch_mode[0] == 0 and self.stitch_mode[1] == 1 and self.stitch_mode[2] == 1): 
			
			if self.vir_pan_val < 0 or self.vir_pan_val > 100:
				printlog(msg="Virtual pan exceeds ranges for CR mode", 
					msg_type="WARN")
				return None, None, None
			
			x_offset = int((self.vir_pan_val*(int(self.pano_size_CR[0] - self.Window_size[0]))/100.))			
			coord[0]= (coord[0]*self.Window_size[0]) + x_offset
			coord[1]= coord[1]*self.pano_size_CR[1]
			coord = [int(coord[0]), int(coord[1])]
			Coord_stitch = coord

			# Look for origin, cheching if coord is inside in any contour
			if self.CR_CtnC is not None: # If there's center camera calibration
				if cv2.pointPolygonTest(np.array(self.CR_CtnC), tuple(coord), True) >= 0 : 
					Origin = "C"
					if self.cachedMCR_INV is not None:
						Coord_src = get_projection_point_dst((coord[0], coord[1], 1), self.cachedMCR_INV)
						Coord_src[1] -= self.source_C_corners[0][1]
					if self.cachedMCR_INV is None: 
						printlog(msg="CR - Variable cachedMCR_INV no defined", 
							msg_type="WARN")
					Coord_src = [int(Coord_src[0]), int(Coord_src[1])]
					return Origin, Coord_src, Coord_stitch

			if self.CR_CtnR is not None: # If there's right camera calibration
				if cv2.pointPolygonTest(np.array(self.CR_CtnR), tuple(coord), True) >= 0 : 
					Origin = "RR"
					if (self.cachedMCR_INV is not None) or (self.cachedHCR_INV is not None):
						Coord_src = get_projection_point_dst((coord[0], coord[1], 1), self.cachedMCR_INV)
						Coord_src = get_projection_point_dst((Coord_src[0], Coord_src[1], 1), self.cachedHCR_INV)
					if self.cachedMCR_INV is None: 
						printlog(msg="CR - Variable cachedMCR_INV no defined", 
							msg_type="WARN")
					if self.cachedHCR_INV is None: 
						printlog(msg="CR - Variable cachedHCR_INV no defined", 
							msg_type="WARN")
					Coord_src = [int(Coord_src[0]), int(Coord_src[1])]
					return Origin, Coord_src, Coord_stitch

			return None, None, Coord_stitch

		# ---------------------------------------------------------------------
		return None, None, None

	def set_proj_cnts(self, cams_params=None):

		""" Draws surface projections in stitching space
		Args:
			cams_params: `dic` dictionary with cameras configurations parameters
		Returns:
		"""

		# Re-assign information
		self.LC_CtnL = self.LC_CtnC = None
		self.CR_CtnC = self.CR_CtnR = None
		self.LCR_CtnL = self.LCR_CtnC = self.LCR_CtnR = None

		# Print information
		printlog(msg="Re-calculation surface projection contour in stitching space", 
			msg_type="INFO")

		if cams_params is None:
			printlog(msg="No camera parameters specified, contours projection no possible", 
				msg_type="WARN")
			return
		else:
			self.PP_CntL = cams_params["LL"]["waypoint_area"] if cams_params["LL"] is not None else None
			self.PP_CntR = cams_params["RR"]["waypoint_area"] if cams_params["RR"] is not None else None
			self.PP_CntC = cams_params["C"]["waypoint_area"] if cams_params["C"] is not None else None
			
			if self.PP_CntL is None: 
				printlog(msg="Parameters for left camera no defined", 
					msg_type="WARN")
			if self.PP_CntC is None: 
				printlog(msg="Parameters for Center camera no defined", 
					msg_type="WARN")
			if self.PP_CntR is None: 
				printlog(msg="Parameters for Right camera no defined", 
					msg_type="WARN")
	
			# -----------------------------------------------------------------
			# For Left and Center = LC

			# For Center contour = C
			if (self.cachedHLC is not None) and (self.PP_CntC is not None):
				self.LC_CtnC = np.array(self.PP_CntC).copy()
				for idx, pt in enumerate(self.LC_CtnC):
					pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHLC)
					self.LC_CtnC[idx] = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLC)
				M = cv2.moments(np.array(self.LC_CtnC))
				if M['m00']: self.LC_CtnC_Mc = [int(M['m10']/M['m00']), int(M['m01']/M['m00'])]

			# For Left contour = L
			if (self.y_offsetLC is not None) and (self.cachedMLC is not None) and (self.PP_CntL is not None):
				self.LC_CtnL = np.array(self.PP_CntL).copy()
				for idx, pt in enumerate(self.LC_CtnL):
					pt[1] += self.source_L_LC_corners[0][1]
					self.LC_CtnL[idx] = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLC)
				M = cv2.moments(np.array(self.LC_CtnL))
				if M['m00']: self.LC_CtnL_Mc = [int(M['m10']/M['m00']), int(M['m01']/M['m00'])]

			# Print any error
			if self.y_offsetLC is None: 
				printlog(msg="CR - No y_offsetLC variable defined", 
					msg_type="ERROR")
			if self.cachedHLC is None: 
				printlog(msg="CR - No cachedHLC variable defined", 
					msg_type="ERROR")
			if self.cachedMLC is None: 
				printlog(msg="CR - No cachedMLC variable defined",
					msg_type="ERROR")
			if self.PP_CntL is None: 
				printlog(msg="LC - Parameters for left camera no defined", 
					msg_type="ERROR")
			if self.PP_CntC is None: 
				printlog(msg="LC - Parameters for Center camera no defined", 
					msg_type="ERROR")

			# Report process information
			if not self.LC_CtnC is None and not self.LC_CtnL is None:
				printlog(msg="LC - Projections in stitching space calculated", 
					msg_type="INFO")
			elif not self.LC_CtnC is None or not self.LC_CtnL is None:
				printlog(msg="LC - Projections in stitching space partially calculated", 
					msg_type="WARN")
			else:
				printlog(msg="LC - Projections in stitching space no calculated", 
					msg_type="ERROR")

			# -----------------------------------------------------------------
			# For Center and Right = CR

			# For Right contour = R
			if self.cachedHCR is not None and self.cachedMCR is not None and self.PP_CntR is not None :
				self.CR_CtnR = np.array(self.PP_CntR).copy()
				for idx, pt in enumerate(self.CR_CtnR):
					pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHCR)
					self.CR_CtnR[idx] = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMCR)
				M = cv2.moments(np.array(self.CR_CtnR))
				if M['m00']: self.CR_CtnR_Mc = [int(M['m10']/M['m00']), int(M['m01']/M['m00'])]

			# For Center contour = C
			if self.y_offsetCR is not None and self.cachedMCR is not None and self.PP_CntC is not None :
				self.CR_CtnC = np.array(self.PP_CntC).copy()
				for idx, pt in enumerate(self.CR_CtnC):
					pt[1] += self.source_C_corners[0][1]
					self.CR_CtnC[idx] = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMCR)
				M = cv2.moments(np.array(self.CR_CtnC))
				if M['m00']: self.CR_CtnC_Mc = [int(M['m10']/M['m00']), int(M['m01']/M['m00'])]

			# Print any error
			if self.cachedHCR is None: 
				printlog(msg="CR - No cachedHCR variable defined", 
					msg_type="ERROR")
			if self.cachedMCR is None: 
				printlog(msg="CR - No cachedMCR variable defined", 
					msg_type="ERROR")
			if self.y_offsetCR is None: 
				printlog(msg="CR - No y_offsetCR variable defined", 
					msg_type="ERROR")
			if self.PP_CntC is None: 
				printlog(msg="CR - Parameters for Center camera no defined",
					msg_type="ERROR")
			if self.PP_CntR is None: 
				printlog(msg="CR - Parameters for Right camera no defined", 
					msg_type="ERROR")
				
			# Report process information
			if self.CR_CtnC is not None and self.CR_CtnR is not None:
				printlog(msg="CR - Projections in stitching space calculated", 
					msg_type="INFO")
			elif self.CR_CtnC is not None or self.CR_CtnR is not None:
				printlog(msg="CR - Projections in stitching space partially calculated", 
					msg_type="WARN")
			else:
				printlog(msg="CR - Projections in stitching space no calculated", 
					msg_type="ERROR")

			# -----------------------------------------------------------------
			# For Left and Center and Right = LCR

			if (self.cachedHLCR is not None) and (self.cachedHCR is not None) and (self.cachedMLCR is not None):
				
				# For Right contour = R
				if (self.PP_CntR is not None) and (self.cachedHCR is not None) and (self.cachedHLCR is not None) and (self.cachedMLCR is not None):
					self.LCR_CtnR = np.array(self.PP_CntR).copy()
					for idx, pt in enumerate(self.LCR_CtnR):
						pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHCR)
						pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHLCR)
						pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLCR)
						pt[1] -= self.in_limitsLCR[0]
						self.LCR_CtnR[idx] = pt
					M = cv2.moments(np.array(self.LCR_CtnR))
					if M['m00']: self.LCR_CtnR_Mc = [int(M['m10']/M['m00']), int(M['m01']/M['m00'])]

				# For Center contour = C
				if (self.PP_CntC is not None) and (self.cachedHLCR is not None) and (self.cachedMLCR is not None):
					self.LCR_CtnC = np.array(self.PP_CntC).copy()
					for idx, pt in enumerate(self.LCR_CtnC):
						pt[1] += self.source_C_corners[0][1]
						pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedHLCR)
						pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLCR)
						pt[1] -= self.in_limitsLCR[0]
						self.LCR_CtnC[idx] = pt	
					M = cv2.moments(np.array(self.LCR_CtnC))
					if M['m00']: self.LCR_CtnC_Mc = [int(M['m10']/M['m00']), int(M['m01']/M['m00'])]

				# For Left contour = L
				if (self.PP_CntL is not None) and (self.cachedMLCR is not None):
					self.LCR_CtnL = np.array(self.PP_CntL).copy()
					for idx, pt in enumerate(self.LCR_CtnL):
						pt[1] += self.source_L_corners[0][1]
						pt = get_projection_point_dst((pt[0], pt[1], 1), self.cachedMLCR)
						pt[1] -= self.in_limitsLCR[0]
						self.LCR_CtnL[idx] = pt	
					M = cv2.moments(np.array(self.LCR_CtnL))
					if M['m00']: self.LCR_CtnL_Mc = [int(M['m10']/M['m00']), int(M['m01']/M['m00'])]

			# Print any error
			if self.cachedHLCR is None: 
				printlog(msg="LCR - No cachedHLCR variable defined", msg_type="ERROR")
			if self.cachedHCR  is None: 
				printlog(msg="LCR - No cachedHCR variable defined", msg_type="ERROR")
			if self.cachedMLCR is None: 
				printlog(msg="LCR - No cachedMLCR variable defined", msg_type="ERROR")
			if self.PP_CntL is None: 
				printlog(msg="LCR - Parameters for Left camera no defined", msg_type="ERROR")
			if self.PP_CntC is None: 
				printlog(msg="LCR - Parameters for Center camera no defined", msg_type="ERROR")
			if self.PP_CntR is None: 
				printlog(msg="LCR - Parameters for Right camera no defined", msg_type="ERROR")

			# Report process information
			if self.LCR_CtnL is not None and self.LCR_CtnC is not None and self.LCR_CtnR is not None :
				printlog(msg="LCR - Projections in stitching space calculated", msg_type="INFO")
			elif self.LCR_CtnL is not None or self.LCR_CtnC is not None or self.LCR_CtnR is not None :
				printlog(msg="LCR - Projections in stitching space partially calculated", msg_type="WARN")
			else:
				printlog(msg="LCR - Projections in stitching space no calculated", msg_type="ERROR")

		# ---------------------------------------------------------------------
		# print("\tNew projections in stitching space saved\n")

		# Save stitcher configuration
		# self.save_stitcher(abs_path = self.path_config_file, quite = True)

	def get_virtual_pan(self, img_src, pan_value=0):

		# Some variables assignation
		pano_size = img_src.shape[:2]
		img_dts = img_src

		# Virtual pan assignation
		self.vir_pan_val = pan_value

		# For stitching mode Left Camera + Center Camera + Righ Camera = "LCR"
		if (self.stitch_mode[0] == 1 and self.stitch_mode[1] == 1 and self.stitch_mode[2] == 1): 
					
			# Offset in X axis to move ROI
			x_offset = 0
			if pan_value >= 0 and pan_value <= 100: 
				x_offset = int((pan_value*(pano_size[1] - int(self.LCR_Cc[0] + self.Window_size[0]/2)))/100.)
			elif pan_value < 0 and pan_value >= -100:
				x_offset = int((pan_value*(int(self.LCR_Cc[0] - self.Window_size[0]/2))/100.))

			# Values to select ROI (Region of interest) 
			rect_x1 = int(self.LCR_Cc[0] - self.Window_size[0]/2) + x_offset 
			rect_x2 = rect_x1 + self.Window_size[0]
			rect_y1 = 0; rect_y2 = pano_size[0]

			if pan_value > 100 or pan_value < -100:
				img_dts = self.image_resize(image = img_src, width  = self.Window_size[0], height = self.Window_size[1], inter  = self.warp_flags)
				cv2.rectangle(img_src, (0, 0), (pano_size[1], pano_size[0]), (0, 255, 0), 2)
			else:
				img_dts = img_src[rect_y1: rect_y2, rect_x1: rect_x2]
				cv2.rectangle(img_src, (rect_x1-2, rect_y1-2), (rect_x2+1, rect_y2+1), (0, 0, 255), 2)

			# Resize window to desired size
			return cv2.resize(src = img_dts, dsize = self.Window_size ,interpolation = self.warp_flags)

		# For stitching mode Left Camera + Center Camera = "LC"
		elif (self.stitch_mode[0] == 1 and self.stitch_mode[1] == 1 and self.stitch_mode[2] == 0): 
			
			x_offset = 0
			if pan_value < 0 and pan_value >= -100:
				x_offset = int((pan_value*(int(pano_size[1] - self.Window_size[0]))/100.))

			# Values to select ROI (Region of interest)
			rect_x1 = int(pano_size[1] - self.Window_size[0]) + x_offset 
			rect_x2 = rect_x1 + self.Window_size[0]
			rect_y1 = 0; rect_y2 = pano_size[0]

			if pan_value > 0 or pan_value < -100:
				img_dts = self.image_resize(image = img_src, width  = self.Window_size[0], height = self.Window_size[1], inter  = self.warp_flags)
			else:
				img_dts = img_src[rect_y1: rect_y2, rect_x1: rect_x2]

			# Resize window to desired size
			return cv2.resize(src = img_dts, dsize = self.Window_size ,interpolation = self.warp_flags)

		# For stitching mode Center Camera + Right Camera = "CR"
		elif (self.stitch_mode[0] == 0 and self.stitch_mode[1] == 1 and self.stitch_mode[2] == 1): 
			
			x_offset = 0
			if pan_value >= 0 and pan_value <= 100:
				x_offset = int((pan_value*(int(pano_size[1] - self.Window_size[0]))/100.))

			# Values to select ROI (Region of interest)
			rect_x1 = x_offset 
			rect_x2 = rect_x1 + self.Window_size[0]
			rect_y1 = 0; rect_y2 = pano_size[0]

			if pan_value < 0 or pan_value > 100:
				img_dts = self.image_resize(image = img_src, width  = self.Window_size[0], height = self.Window_size[1], inter  = self.warp_flags)
			else:
				img_dts = img_src[rect_y1: rect_y2, rect_x1: rect_x2]

			# Resize window to desired size
			return cv2.resize(src = img_dts, dsize = self.Window_size ,interpolation = self.warp_flags)

		# If no condition return input
		return img_src

	def image_resize(self, image, width=None, height=None, inter=cv2.INTER_AREA):

		# initialize the dimensions of the image to be resized and
		# grab the image size
		dim = None
		(h, w) = image.shape[:2]

		# if both the width and height are None, then return the
		# original image
		if width is None and height is None:
			return image

		# check to see if the width is None
		if width is None:
			# calculate the ratio of the height and construct the
			# dimensions
			r = height / float(h)
			dim = (int(w * r), height)

		# otherwise, the height is None
		else:
			# calculate the ratio of the width and construct the
			# dimensions
			r = width / float(w)
			dim = (width, int(h * r))

		# resize the image
		resized = cv2.resize(image, dim, interpolation = inter)

		# resize the image
		if width is not None and height is not None: 
			background = np.zeros((height, width, 3), np.uint8)
			y_pos = int((height*0.5)-(resized.shape[0]*0.5))
			background[y_pos: y_pos + resized.shape[0], 0:resized.shape[1]] = resized
			return background
			
		# return the resized image
		return resized

# =============================================================================
def get_projection_point_dst(coords_src, M):

    """  Gets the coordinate equivalent in surface projection space from original 
		view space 
    Args:
        coords_src: `tuple`  coordinate in the original image space
        M: `cv2.math` rotation matrix from original image to surface
			projection space
    Returns:
        coords_src: `tuple`  projected coordinate in original view space
    """

    coords_dst = np.matmul(M, coords_src)
    coords_dst = coords_dst / coords_dst[2]
    coords_dst = [int(coords_dst[0]), int(coords_dst[1])]

    return coords_dst

def get_extreme_contour_corners(contour):

    """ Calculate extreme corners of contour which is contained into an image
        outer and inner limits will be returned as well
    Args:
        contour: `numpy.ndarray` list of points of contour
    Returns:
        out_limits: `numpy.ndarray` list of outer extreme limits of contour
        in_limits: `numpy.ndarray` list of inner limits of contour
    """

    # Extract x and y axis coordinates
    x_cords = []
    y_cords = []
    for pt in contour:
        x_cords.append(pt[0])
        y_cords.append(pt[1])

    # Find out limits of contour
    out_limits = [min(y_cords), max(y_cords), min(x_cords), max(x_cords)]

    # Find Corners of mask shape
    d_min_ls = d_min_rs = d_min_li = d_min_ri =  max(out_limits)
    pt_ls = pt_rs = pt_li = pt_ri = None

    for pt in contour:
        d_ls = math.sqrt((pt[0])**2+(pt[1])**2)
        d_li = math.sqrt((pt[0])**2+(out_limits[1]-pt[1])**2)

        d_rs = math.sqrt((out_limits[3]-pt[0])**2+(pt[1])**2)
        d_ri = math.sqrt((out_limits[3]-pt[0])**2+(out_limits[1]-pt[1])**2)

        if d_ls <= d_min_ls:
            d_min_ls = d_ls
            pt_ls = pt  
        if d_rs <= d_min_rs:
            d_min_rs = d_rs
            pt_rs = pt 
        if d_li <= d_min_li:
            d_min_li = d_li
            pt_li = pt   
        if d_ri <= d_min_ri:
            d_min_ri = d_ri
            pt_ri = pt  

    corners = [pt_ls, pt_rs, pt_li, pt_ri]

    # Find in limits of contour
    yc = int(min(y_cords) + (max(y_cords) - min(y_cords))*0.5)
    xc = int(min(x_cords) + (max(x_cords) - min(x_cords))*0.5)

    in_limits = []

    # superior inner limit 
    dmin_sup = dmin_inf = max(out_limits)
    inner_sup_y = 0
    inner_inf_y = max(y_cords)
    for y_pt in y_cords:
        if y_pt<=yc:
            d = yc - y_pt
            if d < dmin_sup:
                dmin_sup = d
                inner_sup_y = y_pt
        if y_pt>yc:
            d = y_pt - yc 
            if d < dmin_inf:
                dmin_inf = d
                inner_inf_y = y_pt

    # superior inner limit 
    dmin_left = dmin_right = max(out_limits)
    inner_left_x = 0
    inner_right_x = max(x_cords)
    for x_pt in x_cords:
        if x_pt <= xc:
            d = xc - x_pt
            if d < dmin_left:
                dmin_left = d
                inner_left_x = x_pt
        if x_pt > xc:
            d = x_pt - xc 
            if d < dmin_right:
                dmin_right = d
                inner_right_x = x_pt

    in_limits = [inner_sup_y, inner_inf_y, inner_left_x, inner_right_x]

    return out_limits, in_limits, corners

def is_cv3(or_better=False):
    # grab the OpenCV major version number
    major = get_opencv_major_version()

    # check to see if we are using *at least* OpenCV 3
    if or_better:
        return major >= 3

    # otherwise we want to check for *strictly* OpenCV 3
    return major == 3

def get_opencv_major_version(lib=None):
    # if the supplied library is None, import OpenCV
    if lib is None:
        import cv2 as lib

    # return the major version number
    return int(lib.__version__.split(".")[0])

# =============================================================================



