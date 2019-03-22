Jacky Chung and Yi Zhang
## Purpose
In this competition we demonstrate how to use ARTag recognition, mapping, navigation, and an additional webcam for line following. A majority of this is the same from [competition 2](https://github.com/jackykc/comp2/blob/master/README.md). The new task introduced is eight parking spots where we have to signal through LED and sound the parking spot that has an ARTag, one that was randomly chosen at the start of the round, and one that has the same shape as the green one from task two.

## Prerequisites
* Kobuki Turtlebot with an Asus Xtion Pro and a usb camera
* Ubuntu 16.04
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Desktop or Desktop-Full)
* [Turtlebot](http://wiki.ros.org/action/show/Robots/TurtleBot), [Kobuki Packages](http://wiki.ros.org/kobuki) and [camera](http://wiki.ros.org/openni_camera)
* [cv_camera](http://wiki.ros.org/cv_camera)
  ```
  sudo apt-get install ros-kinetic-turtlebot
  sudo apt-get install ros-kinetic-kobuki
  sudo apt-get install ros-kinetic-kobuki-core
  sudo apt-get install ros-kinetic-openni2-camera
  sudo apt-get install ros-kinetic-openni2-launch
  sudo apt-get install ros-kinetic-cv-camera
  ```
## Resources used
HSV Thresholding
https://docs.opencv.org/3.1.0/df/d9d/tutorial_py_colorspaces.html

Contour Detection
https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html

AR Tracking 
http://wiki.ros.org/ar_track_alvar

## Execution:
1. Build and source setup.bash
   ```
   (In catkin_ws directory after cloning repo into catkin_ws/src)
   catkin_make
   source ./devel/setup.bash
   ```
1. Connect to the Kobuki, Asus Xtion Pro and usb camera
1. Place Kobuki on race track with usb camera looking at the track
1. Startup all required nodes
   `roslaunch comp3 all.launch (will launch the below nodes)`
   * kobuki base
   * 3dsensors
   * bottom camera
   * amcl, map server, move base
   * ar_track_alvar
1. Start the competition three node `roslaunch comp3 comp3.launch rp:=${n}`
  Where `${n}` is the corresponding number of the randomly chosen parking spot
  
## Concepts and code

* State Machine
![alt text](https://raw.githubusercontent.com/jackykc/comp3/master/comp3sm.png)

```
1. In the GO state, the turtlebot does line tracking.
2. Upon seeing a red line, we move to a stop state, incrementing 
   the number of stops it has seen
3. After stopping, it will then move onto one of the four tasks
   or the finish state after the last red line has been seen
4. Task four occurs before task 3.
```
(With the exception of Task 4, the rest are kept the [same](https://github.com/jackykc/comp2/blob/master/README.md))
* GO (Line Tracking)
  1. Convert each frame into HSV format.
  1. Use opencv’s inRange function to filter out the desired color.
  1. Crop out the frame’s most top part.
  1. Use opencv’s moments function to to find the centroid of color in each frame. 
  1. Calculate the distance between color regin and the center of the frame which is the error that the turtlebot is off from     the course.
  1. Use P control to convert the error to how much the turtlebot needs to turn.
* Task 1 (Count objects with a red bottom at location 1)
  1. Threshold to keep only the red pixels of the image
  1. Blur threshold image
  1. Find and count contours larger than a specified minimum size from blurred image
* Task 2 (Count objects at location 2 and determine green shape)
  1. Create two thresholded images, one for red and one for green
  1. Find contours bigger than a specified minimum size for both images
  1. Count the returned contours
  1. Get the shape of the green contour using the number of sides of the contour
* Task 3 (Find shape from location 3 that matches the green one from location 2)
  * For each of the stop lines in task three:
    1. Threshold to keep only the red pixels of the image
    1. Blur thresholded image
    1. Get the shape of the largest red contour using the number of sides
    1. Match the shape with the one obtained from task two
* Task 4 (Visit each parking spot, signaling if the parking spot has and ARTag, the same shape as that of location two, or was randomly chosen at the start of the run.)
  1. Set current pose of the robot for the amcl node (This corner of the room has already been mapped)
  1. Visit each of the parking spots that has been waypointed before
  1. Check each spot and signal for an ARTag, same shape as the green one from task two, or called spot
      * If /ar_pose_marker is publishing an AR tag at current parking spot, we have found the ARTag
      * If the waypoint index matches that of what was random chosen, we have found the called spot
      * Use same method from task 3 to find the parking spot with the correct shape
  1. When the three spots have been signalled, or if all parking spots are visited, go to the exit ramp
  
* AR tag recognition
This process is done by the ar_track_alvar node, we subscribe to “/ar_pose_marker” and store the current marker pose.

# Code explanations (Only task four differs)
[Competition Two code explanations included here for the remaining tasks](https://github.com/jackykc/comp2/blob/master/README.md#code-explanations)

[Get shape at parking spot (task four)](https://github.com/jackykc/comp3/blob/master/src/comp3.py#L227)
``` python
def detect_4(image):
    ...
    # threshold for red colors
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    thresh_red = mask_red
    kernel = numpy.ones((3,3),numpy.float32)/25
    thresh_red = cv2.filter2D(thresh_red,-1,kernel)

    # filter for contours over a certain size
    _, contours_red, hierarchy = cv2.findContours(thresh_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red = list(filter(lambda c: c.size > 40, contours_red))
    vertices = get_vertices(contours_red)
    
    count = clamp_count(len(contours_red))
    # return the shape of the largest contour
    return masked, count, get_shape_id(vertices)
```
[Visit each parking spot for location 4 (task four)](https://github.com/jackykc/comp3/blob/master/src/comp3.py#L602)
``` python
# set initial pose in map
start_pose = initial_pose(waypoints[10])
initial_pose_pub.publish(start_pose)

# turn robot to help localize map
while rospy.Time.now()<wait_time:
    self.twist.linear.x = 0
    self.twist.angular.z = 1.6
    cmd_vel_pub.publish(self.twist)
wait_time = rospy.Time.now() + rospy.Duration(1.2)
while rospy.Time.now()<wait_time:
    self.twist.angular.z = 1.6
    cmd_vel_pub.publish(self.twist)
...
...
# visit each of the eight parking spots
for index, pose in enumerate(waypoints[0:8]):
    goal = goal_pose(pose)

    client.send_goal(goal)
    client.wait_for_result()
    ...
    # for each way point check if the index of the waypoint matches what was called at the start
    # as well as if current_marker_pose exists (it exists when we see an ARTag)
    if current_marker_pose is not None or (rp_id == index):
        if current_marker_pose:
            ar_detected = True
        if rp_id == index:
            random_detected = True
    ...
    
# go to exit ramp
goal = goal_pose(waypoints[8])
client.send_goal(goal)
client.wait_for_result()
```

Video:
https://www.youtube.com/watch?v=SQwM5Pcp4Fw&feature=youtu.be

