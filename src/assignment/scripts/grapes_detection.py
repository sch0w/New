from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np
import datetime
import rospy
import cv2


class image_listener:

    def __init__(self, camera_feed: str):
        # Enable OpenCV with ROS
        self.bridge = CvBridge()
        self.image_counter = 0    # tracks the no of images taken
        # Subscribe to front camera feed
        # "/thorvald_001/kinect2_front_camera/hd/image_color_rect"
        self.image_sub = rospy.Subscriber(camera_feed,
                                          Image, self._handle_feed)

    def _handle_feed(self, data):

        try:
            # ROS Image to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as cv_err:
            print("Exception while converting ROS to OpenCV:", cv_err)
        else:
            self.image_counter += 1
            self._preprocess_image(cv2_img)

    def _preprocess_image(self, cv2_img):
        img_noBackground = self._remove_background(cv2_img) # remove background on start
        img_removeVines = self.removeVines(img_noBackground)
        print("Count of images", self.image_count)
        self.saveImage(img_removeVines)
    
    
    def _remove_background(self, image):
        # Insp[ired from -> https://github.com/TheMemoryDealer/Robot-Programming-CMP9767M/blob/main/weeder/src/vision.py]
        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert color space
        self.saveImage(HSVimage)
        # between values for thresholding
        min = np.array([35, 000, 000]) 
        max = np.array([180, 253, 255]) 
        mask = cv2.inRange(HSVimage, min, max) # threshold
        bunch_image = cv2.bitwise_and(HSVimage, HSVimage, mask=mask) # obtain threshold result
        im_NoBackground = cv2.cvtColor(bunch_image, cv2.COLOR_HSV2BGR) # reconvert color space for publishing
        #cv2.imshow("Removed background", im_NoBackground)
        #cv2.waitKey(0) 
        return im_NoBackground


    def removeVines(self, image):
        HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   # convert color space for thresholding
        #cv2.imshow("HSVImage", HSVimage)
        #cv2.waitKey(0) 
        # mask out vine - values found by using hsv_range_detector.py
        # inspired by https://www.youtube.com/watch?v=We6CQHhhOFo&t=136s  -> ROS and OpenCv for beginners | Blob Tracking and Ball Chasing with Raspberry Pi by Tiziano Fiorenzani
        min = np.array([90, 000, 40])
        max = np.array([255, 255, 255]) 
        vinemask = cv2.inRange(HSVimage, min, max) # threshold
        #cv2.imshow("vinemask", vinemask)
        #cv2.waitKey(0) 
        #self.saveImage(vinemask)

        # Remove odd small spots
        # Inspired by -> https://stackoverflow.com/a/42812226
        dummy_image = vinemask.astype(np.uint8) # reconvert to uint8
        #find all your connected components (white blobs in your image)
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(dummy_image, connectivity=8)
        #connectedComponentswithStats yields every seperated component with information on each of them, such as size
        #the following part is just taking out the background which is also considered a component, but most of the time we don't want that.
        sizes = stats[1:, -1]; nb_components = nb_components - 1
        # minimum size of particles we want to keep (number of pixels)
        #here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever
        min_size = 60 # Value found through trial and error - since we are donig this pre dilation we need ot pick up smaller elements
        #answer image
        vinemask_updated = np.zeros((output.shape))
        #for every component in the image, you keep it only if it's above min_size
        for i in range(0, nb_components):
            if sizes[i] >= min_size:
                vinemask_updated[output == i + 1] = 255

        vinemask_updated = vinemask_updated.astype(np.uint8) # reconvert to uint8
        #cv2.imshow("vinemask updated", vinemask_updated)
        #cv2.waitKey(0) 

        # Increase size of remaining pixels
        vinemask_updated = cv2.dilate(vinemask_updated, np.ones((15, 15)), iterations = 1) # expand mask
        #cv2.imshow("diliated", vinemask_updated)
        #cv2.waitKey(0) 

        # Add kernal to complete the morphEx operation using morph_elispse (simular shape to grapes)
        # Inspired by -> https://www.pyimagesearch.com/2021/04/28/opencv-morphological-operations/
	    # construct a eliptic kernel (same shape as grapes) from the current size and then apply an "opening" operation to close the gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        morph_vinemask = cv2.morphologyEx(vinemask_updated, cv2.MORPH_OPEN, kernel)
        #cv2.imshow("MorphEx vinemask", morph_vinemask)
        #cv2.waitKey(0)

        # obtain threshold result
        grapeBunchImage = cv2.bitwise_and(HSVimage, HSVimage, mask=morph_vinemask) 
        #cv2.imshow("grapeBunchImage with MorphEx ANDED HSV image", grapeBunchImage)
        #cv2.waitKey(0) 

        # Detect the keypoints of the grape bunches in the image and count them
        im_detectGrapes_with_keypoints, keypoints = self.detectGrapes(grapeBunchImage, morph_vinemask)
        #cv2.imshow("Final Grape bunch Image", im_detectGrapes_with_keypoints)
        #cv2.waitKey(0)
        #self.saveImage(im_detectGrapes_with_keypoints)
        
        return im_detectGrapes_with_keypoints


    def detectGrapes(self, image, mask):
        global taken_image

        grape_bunch_mask=cv2.bitwise_not(mask) # invert as blob detector will look for black pixels as ours is white

        # create the small border around the image. As the robot will move forwards down the row then don't catch the right border
        # because the the nexct image the left border will catch any overlap and register it (hopefully!!)
        # If the robot is too close this will work and if far enough away the border wont be required top/bottom
        # Guide: https://stackoverflow.com/questions/53064534/simple-blob-detector-does-not-detect-blobs
        grape_bunch_mask=cv2.copyMakeBorder(grape_bunch_mask, top=1, bottom=1, left=1, right=0, borderType= cv2.BORDER_CONSTANT, value=[255,255,255] ) 

        # REF: https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        # Guide: https://stackoverflow.com/questions/53064534/simple-blob-detector-does-not-detect-blobs 
        # Note: Thorvlad has to stand off so grape bunches are at the image border - this impacts pixel masking params aswell
        params = cv2.SimpleBlobDetector_Params() # initialize detection parameters
        # REF: https://programmerall.com/article/3089974703/  
        params.maxArea = 100000
        params.minInertiaRatio = 0.05
        params.minConvexity = .60
        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs
        keypoints = detector.detect(grape_bunch_mask)
        # Draw detected blobs as red circles. cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (000,000,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #Counts number of grape bunches(keypoint) in image
        grape_bunches_in_image = len(keypoints)
        if(grape_bunches_in_image != 0): # If a blob is detected, print how many there were
            print('Grape bunches = ', grape_bunches_in_image)
        #cv2.imshow("detect keypoints image", im_with_keypoints)
        #cv2.waitKey(0)
        taken_image = True # flags when an image is taken
        return im_with_keypoints, keypoints

    def saveImage(self, image):
        # Save OpenCV2 image as jpeg 
        time = datetime.now()
        imagepath = 'src/assignment/images/grape_bunches'+str(time)+'.jpg' 
        print('saving to ',imagepath)
        cv2.imwrite(imagepath, image)
        
        #imshow("cv2", image)
        rospy.sleep(1)
