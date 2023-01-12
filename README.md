# **Robot Programming Assignment**
*Author: Sean Chow*

*Date: 11/1/2023*

## **Introduction**
This workspace contains an autonomous robot simulation of an agricultural robot (Thorvald) in a virtual vineyard envroment.
The challenge is to count all grape bunches accross the corridors of vines as accurate and as efficient as possible . 
The project involves advanced computer science specializations including computer vision where OpenCV was utilized to operate a set of tools to identify the grape bunches. 

### Autonomous navigation in unknown enviroments
In this simuation, Thorvald utilized a pair of homing beacon to identify a known goal location as it navigates inside an unexplored vineyard.
### Obsticle Avoidance
The BUG2 robot motion planning algorithm is implemented to avoid collisions as Thorvald traverses through the vineyard. 

## **Set Up Your System**

This program is designed and tested in the docker image provided in [https://github.com/LCAS/CMP9767M](https://github.com/LCAS/CMP9767M).

## Install docker engine/docker desktop

### **Linux** (Tested on Ubuntu 22.04.1 LTS)
1. [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
  1. Make sure its properly installed by testing the command `sudo docker run hello-world` in terminal.

2. Follow the [docker image installation guide](https://github.com/LCAS/CMP9767M/wiki/using-the-module-resources-in-docker#using-the-modules-docker-image)

### **Windows** (Tested in docker desktop on both HyverV and WSL2)
### [System requirements for docker desktop](https://docs.docker.com/desktop/install/windows-install/)
- 64 bit processor with Second Level Address Translation (SLAT)
- 4GB system RAM
- BIOS-level hardware virtualization support must be enabled in the BIOS settings

### Download Docker Desktop (slightly tricky)

1. [Install WSL](https://learn.microsoft.com/en-us/windows/wsl/install)
  1. (Optional) Command to ensure WSL2 is enabled:
    wsl.exe --set-default-version 2

2. Download [Ubuntu on Windows](https://apps.microsoft.com/store/detail/ubuntu-on-windows/9NBLGGH4MSV6?hl=en-us&gl=us)

3. Download [docker desktop for windows](https://docs.docker.com/desktop/install/windows-install/) and run `Desktop Installer.exe`
 *If available leave the default check mark ticked for WSL2

4. Please follow the [docker image installation guide](https://github.com/LCAS/CMP9767M/wiki/using-the-module-resources-in-docker#using-the-modules-docker-image)

## Install docker image
[Docker image installation guide](https://github.com/LCAS/CMP9767M/wiki/using-the-module-resources-in-docker#using-the-modules-docker-image)

*Tip: At the end of the work, press [ctrl-C] to stop docker-compose (to stop the container), and to be sure, run `docker-compose down` afterwards to prevent the container using up unnecessary resources*


## Create a catkin workspace

1. Create a catkin workspace in your `~/Desktop` location by following the [ROS catkin tutorials](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

2. Copy the `LCAS/CMP9767M` project into your workspace's `/src` folder (`name_of_your_workspace/src`)

3. Create a folder called `assignment` within your `catkin_ws/src` folder. 
Fork the contents of this [vineyard_simulation](https://github.com/sch0w/vineyard_simulation) repository and clone it into the `assignment` folder 

  Explaination: `assignment` is now a *package* and you have all the files required for this simulation

4. Now that you have all the files required for this simulation. We need to complete the `catkin_make` [process](http://wiki.ros.org/catkin/commands/catkin_make).  `cd ..` out of the folder until you reach the workspace's root file location (~/Desktop/*name_of_your_workspace*). Use `catkin_make` to build your project.

- To ensure the new files are recognised type `roslaunch` , double tab and it should show the package `assignment` as an option. Double tab again and you should see the launch file `launcher.launch`.


## Launch the simulation
To launch Thorvald, type the following in the terminal:

1. `roslaunch grape_bunch_counter grape_bunch_counter.launch`

1. `rosrun grape_bunch_counter homing_beacon.py`
  * use `cd scripts` to locate your scripts folder and type `source devel/setup.bash` if the launch folder is not located/executable.

This calls the package and associated launch and .py files. The `homing_beacon.py` file can only start a few seconds after the launch file to ensure that the sensors in Gazebo(for docker the page in localhost:6080) are fully initialized and Thorvald is taking readings. The launch file itself is very simple and consists of two simple components along with additional launch parameters:

- Node details for relevent .py files

- An <include> to access the bacchus_gazebo vineyard demo launch (by defauly launches the vineyard_small but the map is replacable)

### Update Hokuyo Sensor Profile

The front and back sensor uses the Hokuya laser. It is set to min and max capability and gives an uneven range around Thorvald, especially the sensor laser scan is devided into zones for LEFT, FRONT, RIGHTFRONT, RIGHT etc. The laser has a 45 degrees offset. To adjust this, update the `sensor_hokuyo_laser.xacro` file located in `src/bacchus_lcas/bacchus_gazebo/urdf/bacchus_sensors.xaxro` and change line 21, 22 and 29,30 to:

`min_angle="-0.7854"`

`max_angle="2.3562"`

This will update the laser to scan across a -180 to 180 field of view. You will still have 720 laser data points to work with. 

### Update Thorvald Spawn Point (Optional)

To demonstrate the navigation aspects and collision avoidance, the spawn points for Thorvald can be amended to behind the vine hedge. This can be done be updating the values on lines 36 and 37 in file `src/bacchus_lcas/bacchus_gazebo/urdf/launch/vineyard_demo.launch` to the following:

`<arg name="X_INIT" value="-10.0" unless="$(arg riseholme_dt)"/>`

`<arg name="Y_INIT" value="-10.0" unless="$(arg riseholme_dt)"/>`

This will position the Thorvald robot in the bottom right hand corner near two wall edges and the bottom of the vineyard as per the demonstration video above.


## **Feature Overviews**

## Route Planning Overview

The path planning uses the [BUG2 algorithm](https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/) and a homing beacon system to avoid collisions and get to required points for image taking. 

The algorithm is controlled by being in one of the following state

1. `LOOK_TOWARDS` : Rotates Thorvald towards the `Homing_Beacon`
  - `Homing_Beacon` can be placed anywhere in the map.
  - This project contains `Homing_Beacon` that points to locations where Thorvald should visit and take photos to count grape bunches
  - Once pointed towards the beacon the robot state will be changed to `GOAL_SEEK`

1. `GOAL_SEEK` : Moves Thorvald towards the `Homing_Beacon`
  1. Once encounters obstacle state changes to `WALL_FOLLOW`
  - The collision proximity params are slightly larger than at `WALL_FOLLOW` to avoid getting trapped 

1. `WALL_FOLLOW` : Moves Thorvald out of obsticles
  1. Thorvald will keep moving until it intersects with BUG2's `GOAL"_SEEK` line which it is in line with the next `Homing_Beacon`
  1. State will update to `LOOK_TOWARDS` to again go locate the `Homing_Beacon`

1. `ROTATE_TO"_VINES` : Rotate 90 degrees to face vines
  1. After reaching a goal (`Homing_Beacon`) position, Thorvald will change state to `ROTATE_TO"_VINES` to rotate 90 degrees so the KinectHD camera is directly facing the vines
  1. State will then change to `TAKE_IMAGE`

1. `TAKE_IMAGE` : Capture image and initiate bunch counting procedure
  1.This state happens after `ROTATE_TO"_VINES`, which means Thorvald have already reached `Homing_Beacon` and rotated for the camera to face vines directly

## Grape Bunch Counting Process Overview

The grape bunch counting process is achieved through an imaging pipeline using OpenCV

1. `CV bridge`: Connects/links OpenCV to ROS
2. `cv2.cvtColor(image, cv2.COLOR_BGR2HSV)`: Convert to HSV image, apply thresholds then mask. A useful threshold tool is the [blob_detector.py](https://github.com/tizianofiorenzani/ros_tutorials/blob/master/opencv/include/blob_detector.py) by Tiziano Fiorenzani.
1. Repeat the above process but remove the green vines applying a new threshold
1. We now have an image with lots of smaller white dots. We need to remove this noise. We used `astype(np.uint8)` to convert to unit8, then `cv2.connectedComponentsWithStats(dummy_image, connectivity=8)` to build a list of centroids of all white dots. We then filter and remove any that are below 60 pixels in size (chosen via trial and error).
1. `cv2.dilate(vinemask_updated, np.ones((15, 15)), iterations = 1)` was used to [increase the size](https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html) of the mask points. 
1. We then use `cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))` to apply an elliptical shape to a following morphologyEx process
1. `cv2.morphologyEx(vinemask_updated, cv2.MORPH_OPEN, kernel)` to open the pixels to create larger centroid regions. The kernel size is critical here and we use 5,5. 7,7 opened up the image to far relative to distance image was being taken at and results were poor.

The following stage of the pipeline uses the `cv2.SimpleBlobDetector` to detect the grapes:

1. `cv2.copyMakeBorder(grape_bunch_mask, top=1, bottom=1, left=1, right=0, borderType= cv2.BORDER_CONSTANT, value=[255,255,255] )` to add a small border to the top, bottom, left but not the right hand side *(see Kinect Camera offset to Vines details for why)*
1. `cv2.SimpleBlobDetector_Params`, we add the required parameters. Some params are set as default so need ot be adjust to ensure we can identify the shapes of the grape bunches (and not just circles (circularity) for example). These params were discovered through trial and error across a range of images and lighting conditions, but only on 1 x compute resource. Deploying on alternative compute resources may require some amendment to these thresholds as well as HSV thresholds. 
1. `keypoints = detector.detect(grape_bunch_mask)`, creates a detector object and identifies keypoints in the image to our previously set params.
1. `cv2.drawKeypoints(image, keypoints, np.array([]), (000,000,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)` draws the keypoints onto the image. There is a helper function within the file to save the any images to your local directory. 
1. The final step is the accumulation of the `keypoints` count. We do this for the images taken across the length of the vine and sum the total bunches (keypoints identified in each image) found for a total count of grape bunches. We display this to the terminal for the user.

### Kinect Camera offset to Vines

The distance to the vines is correlated to the field of view of the Kinect_v2 camera. This information can be found in the `src/bacchus_lcas/bacchus_gazebo/urdf/sensors/sensor_kinect_v2_.xaxro` file on line 109. This gives us a value of `<horizontal_fov>${84.1*3.14/180.0}</horizontal_fov>` which is around 1.467 rads for the viewing angle. The length of the vine hedge in the 'world_name:=vineyard_small' is 12 meters (found from visual inspection). At 2m from the hedge we will image 3.60794m of the hedge with the available field of view. Taking an image of the full vine hedge at 2m distance therefore requires 12 / 3.61 = 3.32 images. Or rather the last image will only have 1/3rd of the vine hedge on (if we take an image directly at the start of the hedge with the RHS at the border of the image). We need to be careful that we don't crop the top of the hedge if we are too close.

The images are taken from right through left. As part of the process we use `cv2.SimpleBlobDetector`. The simple blob detector ignores keypoints on the boundary so a border was created with the right side of the border removed (so anything on the RHS would not be counted if it fell on the this line). As the robot moves down the vine row, anything missed out on the RHS is captured on the LHS image. This avoids the issue of double counting boundary grape bunches as long as we are careful with our positioning; however robot distance to hedge and camera FoV should be as accurate as possible to get sensible results.
