#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from kobuki_msgs.msg import Led, Sound
from geometry_msgs.msg import Twist
import cv2, cv_bridge, numpy
import smach
import smach_ros

from geometry_msgs.msg import PoseWithCovarianceStamped

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
import actionlib


global stop, donot_check_time, image_pub, err, cmd_vel_pub, bridge, stop_count, line_lost, led_pub1, led_pub2, sound_pub

global client, waypoints, waypoint_id, current_marker_pose, start_detect
start_detect = False
current_marker_pose = None
waypoint_id = 0 # 0 - 7
'''
shape_id
0 triangle
1 square
2 circle
'''
global shape_id_counts, object_counts, chosen_shape
chosen_shape = "circle"

global rp_id
rp_id = 0
shape_id_counts = {
    "task2": numpy.asarray([0, 0, 0]),
    "task3": numpy.asarray([0 ,0 ,0]),
    "task4": numpy.asarray([0, 0, 0])
 } # green, red (task 2 and 3)
object_counts = {
    "task1": numpy.asarray([0, 0, 0]), # task 1 [1obj, 2obj, 3obj]
    "task2": numpy.asarray([0, 0, 0])  # task 2 [1obj, 2obj, 3obj]
} # task 1 and 2

line_lost = False
stop_count = 0
rospy.init_node('comp3')
err = 0

led_pub1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
led_pub2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
rp_id = rospy.get_param('comp3/rp', 0)

if rp_id == 6:
    rp_id = 7
elif rp_id == 7:
    rp_id = 6
elif rp_id == 8:
    rp_id = 5
else:
    rp_id -= 1

rospy.loginfo(rp_id)
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
client.wait_for_server()

global start, callback_state
start = True
callback_state = 0
'''
0 follow line
1 task 1
2 task 2
3 task 3
'''

def marker_cb(msg):
    if len(msg.markers):
        # print msg.markers

        for marker in msg.markers:
            global current_marker_pose
            current_marker_pose = marker.pose.pose

def joy_callback(msg):
    global start
    if msg.buttons[0] == 1:
        rospy.loginfo("start pressed!")
        start = not start 

def follow_line(msg):
    global stop, donot_check_time, image_pub, err, line_lost, callback_state
    if callback_state != 0:
        # using the other camera
        return
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = numpy.array([0, 0,  242])
    upper_white = numpy.array([170, 50, 256])
    lower_red = numpy.array([130, 132,  110])
    upper_red = numpy.array([200, 256, 256])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    masked = cv2.bitwise_and(image, image, mask=mask_red)
    # image_pub.publish(bridge.cv2_to_imgmsg(masked, encoding='bgr8'))

    # check red line
    h, w, d = image.shape
    search_top = h-70
    search_bot = h-50
    mask_red[0:search_top, 0:w] = 0
    mask_red[search_bot:h, 0:w] = 0
    M = cv2.moments(mask_red)
    if M['m00'] > 0 and rospy.Time.now() > donot_check_time:
        stop = True
        donot_check_time = rospy.Time.now()+rospy.Duration(5)
    if stop:
        if M['m00'] > 0: 
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,255,0), -1)
            image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        return

    # masked = cv2.bitwise_and(image, image, mask=mask)
    # image_pub.publish(bridge.cv2_to_imgmsg(masked, encoding='bgr8'))

    # track white line
    h, w, d = image.shape
    search_top = h-70
    search_bot = h-50
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
        line_lost = False
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        err = cx - w/2
    else:
        line_lost = True
    image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))

def display_led(count):
    global led_pub1, led_pub2
    if count == 0:
        led_pub1.publish(Led(Led.BLACK))
        led_pub2.publish(Led(Led.BLACK))
    elif count == 1:
        led_pub1.publish(Led(Led.BLACK))
        led_pub2.publish(Led(Led.GREEN))
    elif count == 2:
        led_pub1.publish(Led(Led.ORANGE))
        led_pub2.publish(Led(Led.BLACK))
    elif count == 3:
        led_pub1.publish(Led(Led.RED))
        led_pub2.publish(Led(Led.RED))

def detect_1(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #old
    # lower_red = numpy.array([130, 132,  110])
    # upper_red = numpy.array([200, 256, 256])
    
    # as of march 13
    lower_red = numpy.array([130, 132,  110])
    upper_red = numpy.array([180, 255, 255])
    
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    h, w, d = image.shape
    mask_red[:h/2] = 0

    ret, thresh = cv2.threshold(mask_red, 127, 255, 0)
    
    kernel = numpy.ones((9,9),numpy.float32)/25
    thresh = cv2.filter2D(thresh,-1,kernel)
    
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = list(filter(lambda c: c.size > 35, contours))
    cv2.drawContours(image, contours, -1, (0, 0, 255), 3)

    masked = cv2.bitwise_and(image, image, mask=mask_red)

    count = clamp_count(len(contours))
    return masked, count

def detect_2(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = numpy.array([130, 132,  110])
    upper_red = numpy.array([200, 256, 256])
    lower_green = numpy.array([44, 54,  63])
    upper_green = numpy.array([88, 255, 255])
    
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    ret, thresh_red = cv2.threshold(mask_red, 127, 255, 0)

    # thresh_red = mask_red
    thresh_green = mask_green # did not bother doing threshold on green

    kernel = numpy.ones((3,3),numpy.float32)/25
    thresh_red = cv2.filter2D(thresh_red,-1,kernel)
    
    _, contours_green, hierarchy = cv2.findContours(thresh_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    _, contours_red, hierarchy = cv2.findContours(thresh_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_green = list(filter(lambda c: c.size > 70, contours_green))
    contours_red = list(filter(lambda c: c.size > 40, contours_red))
    
    vertices = get_vertices(contours_green)

    cv2.drawContours(image, contours_green, -1, (0,255,0), 3)
    cv2.drawContours(image, contours_red, -1, (0,0,255), 3)

    mask = cv2.bitwise_or(mask_red, mask_green)
    masked = cv2.bitwise_and(image, image, mask=mask)

    count = clamp_count(len(contours_red) + 1)
    return masked, count, get_shape_id(vertices)

def detect_4(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = numpy.array([130, 132,  110])
    upper_red = numpy.array([180, 256, 256])

    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    h, w, d = image.shape
    # mask_red[:h*2/3] = 0
    # ret, thresh_red = cv2.threshold(mask_red, 127, 255, 0)
    thresh_red = mask_red

    kernel = numpy.ones((3,3),numpy.float32)/25
    thresh_red = cv2.filter2D(thresh_red,-1,kernel)

    _, contours_red, hierarchy = cv2.findContours(thresh_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_red = list(filter(lambda c: c.size > 40, contours_red))
    
    vertices = get_vertices(contours_red)

    cv2.drawContours(image, contours_red, -1, (0,0,255), 3)

    
    mask = mask_red
    masked = cv2.bitwise_and(image, image, mask=mask)

    count = clamp_count(len(contours_red))
    return masked, count, get_shape_id(vertices)


def detect_3(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = numpy.array([130, 132,  110])
    upper_red = numpy.array([180, 256, 256])

    # lower_red = numpy.array([0, 205,  38])
    # upper_red = numpy.array([180, 255, 125])

    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    h, w, d = image.shape
    mask_red[:,0:w/5] = 0
    mask_red[:,4*w/5:w] = 0
    mask_red[:h*2/3] = 0
    # ret, thresh_red = cv2.threshold(mask_red, 127, 255, 0)
    thresh_red = mask_red

    kernel = numpy.ones((3,3),numpy.float32)/25
    thresh_red = cv2.filter2D(thresh_red,-1,kernel)

    _, contours_red, hierarchy = cv2.findContours(thresh_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_red = list(filter(lambda c: c.size > 40, contours_red))
    
    vertices = get_vertices(contours_red)

    cv2.drawContours(image, contours_red, -1, (0,0,255), 3)

    
    mask = mask_red
    masked = cv2.bitwise_and(image, image, mask=mask)

    count = clamp_count(len(contours_red))
    return masked, count, get_shape_id(vertices)
        

def clamp_count(count):
    if count < 1:
        return 1
    elif count > 3:
        return 3
    else:
        return count

def get_shape_id(vertices):
    if vertices == 3:
        id = 0
    elif vertices == 4:
        id = 1
    else:
        id = 2

    return id

def get_shape(shape_id):
    if shape_id == 0:
        shape = "triangle"
    elif shape_id == 1:
        shape = "square"
    else:
        shape = "circle"

    return shape

def get_vertices(contours):
    approx = []
    areas = [cv2.contourArea(c) for c in contours]
    if len(areas):
        max_index = numpy.argmax(areas)
        largest_contour = contours[max_index]

        peri = cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, 0.04 * peri, True)
    return len(approx)

def image_callback(msg):
    global callback_state
    global shape_id_counts, object_counts
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    if callback_state == 0:
        pass
        # follow_line(image)
    elif callback_state == 1:
        image, count = detect_1(image)
        object_counts["task1"][count-1] += 1 # count -1 as index starts at 0
        image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        # display_led(count)
    elif callback_state == 2: # reset object_counts in the state
        image, count, shape_id = detect_2(image)
        object_counts["task2"][count-1] += 1
        shape_id_counts["task2"][shape_id] += 1
        image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        # display_led(count)
    elif callback_state == 3:
        image, count, shape_id = detect_3(image)
        shape_id_counts["task3"][shape_id] += 1
        image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
    elif callback_state == 4:
        image, count, shape_id = detect_4(image)
        shape_id_counts["task4"][shape_id] += 1
        image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        
rospy.Subscriber("/joy", Joy, joy_callback)
bridge = cv_bridge.CvBridge()
image_pub = rospy.Publisher('transformed_img', Image, queue_size=1)
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
bottom_cam = rospy.Subscriber('/bottom/rgb/image_raw', Image, follow_line)
marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, marker_cb)

# cmd_vel_pub = rospy.Publisher('/teleop_velocity_smoother/raw_cmd_vel', Twist, queue_size=1)
donot_check_time = rospy.Time.now()
stop = False

class Go(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])
        self.twist = Twist()
        print "start"
    def execute(self, data):
        global stop, err, cmd_vel_pub, stop_count, start
        while not rospy.is_shutdown():
            if stop:
                stop = False
                return 'stop'
            else:
                self.twist.linear.x = 0.3#0.2
                self.twist.angular.z = -float(err) / 200
                cmd_vel_pub.publish(self.twist)

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go', 'task1', 'task2', 'task3', 'task4', 'finish'])
        self.twist = Twist()
    def execute(self, data):
        global stop, cmd_vel_pub, stop_count
        stop_count += 1
        # go a bit further
        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        # determin which it is
        if stop_count == 1:
            return 'task1'
        elif stop_count == 3:
            return 'task2'
        elif stop_count == 4:
            return 'task4'
        elif stop_count == 5:
            wait_time = rospy.Time.now() + rospy.Duration(1)
            while rospy.Time.now()<wait_time:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                cmd_vel_pub.publish(self.twist)
            return 'task3'
        elif stop_count <0:
            return 'finish'
        # regular stop
        wait_time = rospy.Time.now() + rospy.Duration(2)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        return 'go'

class Task1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go'])
        self.twist = Twist()
    def execute(self, data):
        global stop, cmd_vel_pub, callback_state, sound_pub

        wait_time = rospy.Time.now() + rospy.Duration(1.4) 
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1)
        callback_state = 1
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        callback_state = 0
        object_count = numpy.argmax(object_counts["task1"]) + 1
        for i in range(object_count):
            # wait_time_sound = rospy.Time.now() + rospy.Duration(0.5)
            # while rospy.Time.now()<wait_time_sound:
            sound_pub.publish(Sound(0))
            wait_time_sound = rospy.Time.now() + rospy.Duration(1)
            while rospy.Time.now()<wait_time_sound:
                continue
        wait_time = rospy.Time.now() + rospy.Duration(1.2)
        rospy.loginfo("object count" + str(object_count))
        # while not rospy.is_shutdown():
        #     continue
        while rospy.Time.now()<wait_time:
            display_led(object_count)
            self.twist.linear.x = 0
            self.twist.angular.z = -1.5
            cmd_vel_pub.publish(self.twist)

        return 'go'

class Task2(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['go'])
        self.twist = Twist()
    def execute(self, data):
        global stop, cmd_vel_pub, err,  line_lost, callback_state, object_counts, shape_id_counts, sound_pub, chosen_shape
        display_led(0) # clear LED
            
        print 'in task 2'
        # wait_time = rospy.Time.now() + rospy.Duration(1.5)
        # while rospy.Time.now()<wait_time:
        #     self.twist.linear.x = 0.2
        #     self.twist.angular.z = 0
        #     cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1.5)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5
            cmd_vel_pub.publish(self.twist)
        # track the line
        print 'tracking line'
        while (not rospy.is_shutdown()) and (not line_lost):
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 200
            cmd_vel_pub.publish(self.twist)
        # reaches the end, stop for 2 second
        print 'reaches the end'
        callback_state = 2
        wait_time = rospy.Time.now() + rospy.Duration(2)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        callback_state = 0
        # turn back
        wait_time = rospy.Time.now() + rospy.Duration(2.5)
        object_count = numpy.argmax(object_counts["task2"]) + 1
        while rospy.Time.now()<wait_time:
            display_led(object_count)
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5
            cmd_vel_pub.publish(self.twist)
        # track the line
        for i in range(object_count):
            # wait_time_sound = rospy.Time.now() + rospy.Duration(0.5)
            # while rospy.Time.now()<wait_time_sound:
            sound_pub.publish(Sound(0))
            wait_time_sound = rospy.Time.now() + rospy.Duration(1)
            while rospy.Time.now()<wait_time_sound:
                continue
        
        stop = False
        while (not rospy.is_shutdown()) and not stop:
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 200
            cmd_vel_pub.publish(self.twist)
        # stops at red line, return to go state
        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5
            cmd_vel_pub.publish(self.twist)
        stop = False
        chosen_shape = get_shape(numpy.argmax(shape_id_counts["task2"]))
        rospy.loginfo(str(chosen_shape))

        return 'go'


waypoints = [  # <1>
    [(4.4510,  -0.123, 0.0), (0.0, 0.0, -0.4471,  0.89448)], # 1
    [(3.745, -0.521, 0.0), (0.0, 0.0, -0.505, 0.862)], # 2
    [(3.037, -0.8509, 0.0), (0.0, 0.0, -0.482, 0.8756)], # 3 # parking spot 3
    [(2.25, -1.20, 0.0), (0.0, 0.0, -0.4735, 0.9058)], # 4

    [(1.492, -1.613, 0.0), (0.0, 0.0, -0.600, 0.7999)], # 5
    
    [(1.2, -0.86, 0.0), (0.0, 0.0, -0.9119, 0.410)], # 8
    [(2.0882, 0.1414, 0.0), (0.0, 0.0, 0.88392, 0.4676)], # 7
    [(2.9047, 0.4278, 0.0), (0.0, 0.0, 0.8565, 0.516)], # 6
    
    
    [(3.539, 1.4547, 0.0), (0.0, 0.0,  0.890, 0.44798)], # id8 end of course   

    [(2.866, -0.227, 0.0), (0.0, 0.0,  -0.9901, 0.139)], # id9 middle
    [(0.1780, -0.0476, 0.0), (0.0, 0.0,  0.2488, 0.96855)], # id10 start
    [(0.6432, 0.396, 0.0), (0.0, 0.0,  -0.0287, 0.99958)], # id11 fork
    [(1.2163, 0.14463, 0.0), (0.0, 0.0,  -0.2283, 0.9735)], # id12 end of line
]

def goal_pose(pose):  # <2>
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

def initial_pose(pose):
    goal_pose = PoseWithCovarianceStamped()
    goal_pose.pose.pose.position.x = pose[0][0]
    goal_pose.pose.pose.position.y = pose[0][1]
    goal_pose.pose.pose.position.z = pose[0][2]
    goal_pose.pose.pose.orientation.x = pose[1][0]
    goal_pose.pose.pose.orientation.y = pose[1][1]
    goal_pose.pose.pose.orientation.z = pose[1][2]
    goal_pose.pose.pose.orientation.w = pose[1][3]

    return goal_pose

class Task4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go'])
        self.twist = Twist()
    def execute(self, data):
        global stop, cmd_vel_pub, callback_state, sound_pub, client
        global current_marker_pose, stop_count
        global shape_id_counts, chosen_shape, rp_id

        callback_state = 4
        ar_detected = False
        color_detected = False
        random_detected = False
        start_pose = initial_pose(waypoints[10])
        initial_pose_pub.publish(start_pose)
    
        wait_time = rospy.Time.now() + rospy.Duration(1.2)
        while rospy.Time.now()<wait_time:
            # turn
            self.twist.linear.x = 0
            self.twist.angular.z = -1.6
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1.2)
        while rospy.Time.now()<wait_time:
            # turn
            self.twist.linear.x = 0
            self.twist.angular.z = 1.6
            cmd_vel_pub.publish(self.twist)


        goal = goal_pose(waypoints[11]) # fork
        client.send_goal(goal)
        client.wait_for_result()
        goal = goal_pose(waypoints[12]) # end of line
        client.send_goal(goal)
        client.wait_for_result()

        goal = goal_pose(waypoints[9]) # middle
        client.send_goal(goal)
        client.wait_for_result()

        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            display_led(0)

        for index, pose in enumerate(waypoints[0:8]):
            goal = goal_pose(pose)

            client.send_goal(goal)
            client.wait_for_result()

            start_detect = True
            current_marker_pose = None
            wait_time = rospy.Time.now() + rospy.Duration(2)

            is_shape = False
            callback_state = 4

            while rospy.Time.now()<wait_time:
                display_led(0)
                if (numpy.sum(shape_id_counts["task4"]) != 0):
                    current_shape = get_shape(numpy.argmax(shape_id_counts["task4"]))
                    shape_id_counts["task4"][0] = 0
                    shape_id_counts["task4"][1] = 0
                    shape_id_counts["task4"][2] = 0

                if chosen_shape == current_shape:
                    is_shape = True
            if current_marker_pose is not None or (rp_id == index):
                if current_marker_pose:
                    ar_detected = True
                if rp_id == index:
                    print index
                    print rp_id
                    random_detected = True
                
                wait_time = rospy.Time.now() + rospy.Duration(2)
                while rospy.Time.now() < wait_time:
                    sound_pub.publish(Sound(0))
                    if current_marker_pose:
                        display_led(1)
                    elif rp_id == index:
                        display_led(3)
            start_detect = False
            current_marker_pose = None
            if ar_detected and random_detected:
                break

        '''
        object_count = numpy.argmax(object_counts["task1"]) + 1
        for i in range(object_count):
            # wait_time_sound = rospy.Time.now() + rospy.Duration(0.5)
            # while rospy.Time.now()<wait_time_sound:
            sound_pub.publish(Sound(0))
            wait_time_sound = rospy.Time.now() + rospy.Duration(1)
            while rospy.Time.now()<wait_time_sound:
                continue
        '''
        goal = goal_pose(waypoints[8])
        client.send_goal(goal)
        client.wait_for_result()

        stop_count = 4
        callback_state = 0
        return 'go'


class Task3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go'])
        self.twist = Twist()
    def execute(self, data):
        global stop, cmd_vel_pub, callback_state, object_counts, shape_id_counts, chosen_shape, stop_count
        shape_found = False
        for i in range(0, 3):
            # track the line
            stop = False
            while (not rospy.is_shutdown()) and not stop:
                display_led(0) # clear LED
                self.twist.linear.x = 0.1
                self.twist.angular.z = -float(err) / 200
                cmd_vel_pub.publish(self.twist)
            print("see the red line")

            # go a bit further
            wait_time = rospy.Time.now() + rospy.Duration(3)
            while rospy.Time.now()<wait_time:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0
                cmd_vel_pub.publish(self.twist)

            # dont check if we already found it
            if shape_found:
                continue
            # check each one
            wait_time = rospy.Time.now() + rospy.Duration(1.4)
            while rospy.Time.now()<wait_time:
                # turn
                self.twist.linear.x = 0
                self.twist.angular.z = 1.6
                cmd_vel_pub.publish(self.twist)
            wait_time = rospy.Time.now() + rospy.Duration(4)
            callback_state = 3
            while rospy.Time.now()<wait_time:
                # stop
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                cmd_vel_pub.publish(self.twist)
            callback_state = 0
            current_shape = get_shape(numpy.argmax(shape_id_counts["task3"]))
            rospy.loginfo(str(current_shape))
            if (current_shape == chosen_shape) and not shape_found:
                shape_found = True
                wait_time_sound = rospy.Time.now() + rospy.Duration(0.5)
                # while rospy.Time.now()<wait_time_sound:
                sound_pub.publish(Sound(0))
            # turn back to line
            wait_time = rospy.Time.now() + rospy.Duration(1.4)
            while rospy.Time.now()<wait_time:
                self.twist.linear.x = 0
                self.twist.angular.z = -1.5
                cmd_vel_pub.publish(self.twist)
            # wait_time = rospy.Time.now() + rospy.Duration(1)
            # while rospy.Time.now()<wait_time:
            #     # backup
            #     self.twist.linear.x = -0.2
            #     self.twist.angular.z = 0
            #     cmd_vel_pub.publish(self.twist)
        # check last one
        
        # wait_time = rospy.Time.now() + rospy.Duration(3.4)
        # while rospy.Time.now()<wait_time:
        #     self.twist.linear.x = 0.2
        #     self.twist.angular.z = 0
        #     cmd_vel_pub.publish(self.twist)
        # wait_time = rospy.Time.now() + rospy.Duration(1.4)
        # while rospy.Time.now()<wait_time:
        #     # turn
        #     self.twist.linear.x = 0
        #     self.twist.angular.z = 1.5
        #     cmd_vel_pub.publish(self.twist)
        # wait_time = rospy.Time.now() + rospy.Duration(4)
        # callback_state = 3
        # while rospy.Time.now()<wait_time:
        #     self.twist.linear.x = 0
        #     self.twist.angular.z = 0
        #     cmd_vel_pub.publish(self.twist)
        # callback_state = 0
        # current_shape = get_shape(numpy.argmax(shape_id_counts["task3"]))
        # rospy.loginfo(str(current_shape))
        # if (current_shape == chosen_shape) and not shape_found:
        #     shape_found = True
        #     wait_time_sound = rospy.Time.now() + rospy.Duration(0.5)
        #     while rospy.Time.now()<wait_time_sound:
        #         sound_pub.publish(Sound(Sound.BUTTON))
        
        # wait_time = rospy.Time.now() + rospy.Duration(1.4)
        # while rospy.Time.now()<wait_time:
        #     self.twist.linear.x = 0
        #     self.twist.angular.z = -1.5
        #     cmd_vel_pub.publish(self.twist)

        stop = False
        stop_count = -200
        return 'go'

# follower = Follower()
sm = smach.StateMachine(outcomes=['finish'])
with sm:
    # Add states to the container
    smach.StateMachine.add('GO', Go(), 
                 transitions={'stop':'STOP'})
    smach.StateMachine.add('STOP', Stop(), 
                 transitions={'go':'GO', 'task1': 'TASK1', 'task2': 'TASK2', 'task3': 'TASK3', 'task4': 'TASK4', 'finish': 'finish'})
    smach.StateMachine.add('TASK1', Task1(), 
                 transitions={'go':'GO'})
    smach.StateMachine.add('TASK2', Task2(), 
                 transitions={'go':'GO'})
    smach.StateMachine.add('TASK3', Task3(), 
                 transitions={'go':'GO'})
    smach.StateMachine.add('TASK4', Task4(), 
                 transitions={'go':'GO'})

# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

outcome = sm.execute()
rospy.spin()
sis.stop()