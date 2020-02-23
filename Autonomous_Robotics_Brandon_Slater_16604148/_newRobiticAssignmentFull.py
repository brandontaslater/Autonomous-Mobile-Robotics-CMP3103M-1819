import rospy
import numpy
import cv2
import numpy as np
import math
import time
import actionlib

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


# RED, GREEN, BLUE, YELLOW       
class AutonomouseRoboticsAssignment:
    def __init__(self):
                
        # opens the windwow for CV2
        self.bridge = CvBridge()
        cv2.startWindowThread()
        
        # times the program
        self.start_time =  0.0
        self.minutes = 0
        self.seconds = 0
        self.seconds_timer = 0.00
        self.timer_label = ""
        
        # waypoints used to go around the map
        self.map_cordinates = [[0.50, 2.00],
                               [2.00, 4.50],
                               [-4.50, 4.00],
                               [-4.50, -1.00],
                               [2.00, -1.00],
                               [1.50, -4.00],
                               [-3.50, -4.50]]
                               
        # ABOVE, LEFT, BELOW, RIGHT
        self.coordinated_around_colour = [[0.00, 0.00],
                                          [0.00, 0.00],
                                          [0.00, 0.00],
                                          [0.00, 0.00]]
                                          
        self.coordinate_around_original_waypoint = [[0.00, 0.00],
                                                    [0.00, 0.00],
                                                    [0.00, 0.00],
                                                    [0.00, 0.00]]
                            
        # array of booleans for whether the colour has been found 
        self.colours_found = [False, False, False, False]
        
        # array of booleans for whether the colour is being checked if in range 
        self.colours_in_camera = [False, False, False, False]
        
        # stores the current waypoint traveling too 
        self.current_waypoint = [0.00,0.00]        
        
        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # orientation of the robot 
        self.orientation_z = 0.00
        
        # Gets the object head on
        self.rangeOfObstacle = 0.00
        
        # check if complete 
        self.complete = False
        
        # ends the rotation if false                               
        self.end_rotation = False
            
        # object found NOT COLOUR 
        self.object_found = True

        # colour found 
        self.moving_to_waypoint = False
            
        # colour found 
        self.found_colour = False
        
        # for searching 
        self.search_camera = False
        
        # value for which colour is being looked at 
        self.current_index = -1
        
        # if waypoints around the colour have already been assigned 
        self.waypoints_set = False
        
        # X and Y position:
        self.robotposx = 0.00
        self.robotposy = 0.00
        
        # X and Y Test positions:
        self.Potential_X_Robit = 0.00
        self.Potential_Y_Robit = 0.00

        # Publish the command velocity when it see's an object        
        self.publish_teleop = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        
        #self.tf = tf.TransformListener()
        #self.tf_sub = rospy.Subscriber("/tf", PointStamped, self.update_transform)       
        
        # Get the turtlebots view        
        self.laser_scan_sub = rospy.Subscriber("/scan", LaserScan, self.laser_call) 
        
        # Get the turtlebots view        
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_frames)  

        # Allows us to retrieve odometry data such as position and rotation      
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.xy_current_positions)
        
        # allows callback of self.twist throughout for moving the robot        
        self.twist = Twist() 
        
        # CALLS MAIN TO START THE PROGRAM
        self._main()
    
    
    # RED, GREEN, BLUE, YELLOW 
    def image_frames(self, data):
        # Creates the cv2 camera windows 
        cv2.namedWindow("Camera Veiw:", 1) 
        
        # Convert the image message to a BGR image
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')  
        
        # Get the image dimensions
        height, width, depth = self.image.shape
        
        # stores all colour masks
        _colour_masks = [] 
        
        # Create the HSV image
        hsvImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    
        # Create the colour masks
        # Hsv Image, lower and upper for colours in segmentation       
        # RED
        _colour_masks.append(cv2.inRange(hsvImage, numpy.array([0, 200, 30]), numpy.array([5, 255, 150])))

        # GREEN    
        _colour_masks.append(cv2.inRange(hsvImage, numpy.array([60, 200, 30]), numpy.array([80, 255, 200])))

        # BLUE
        _colour_masks.append(cv2.inRange(hsvImage, numpy.array([90, 200, 30]), numpy.array([120, 255, 230])))

        # YELLOW
        _colour_masks.append(cv2.inRange(hsvImage, numpy.array([30, 200, 30]), numpy.array([40, 255, 200])))
        
        #holds the big mask for all colours 
        self.bigMask = cv2.inRange(hsvImage, numpy.array([0,0,0]), numpy.array([0,0,0]))
        
        self.search_top = 1 * height / 4
        self.search_bot = 3 * height / 4        
        
        # RED, GREEN, BLUE, YELLOW order
        _colour_masks[0][0:self.search_top, 0:width] = 0
        _colour_masks[1][0:self.search_bot:height, 0:width] = 0
        _colour_masks[2][0:self.search_bot:height, 0:width] = 0
        _colour_masks[3][0:self.search_bot:height, 0:width] = 0
        
        # Create the mega mask
        # RED, GREEN, BLUE, YELLOW order
        if(self.colours_found[0] == False):
            self.bigMask += _colour_masks[0]
        if(self.colours_found[1] == False):
            self.bigMask += _colour_masks[1]    
        if(self.colours_found[2] == False):
            self.bigMask += _colour_masks[2]
        if(self.colours_found[3] == False):
            self.bigMask += _colour_masks[3]       
        
        # checks to see if the colour is found, if true dont run 
        if self.search_camera:
            #loops through all the colours 
            for i in range(4):
                moment = cv2.moments(_colour_masks[i])
                # checks if no colours are being looked at 
                if not self.colours_in_camera[0] and not self.colours_in_camera[1] and not self.colours_in_camera[2] and not self.colours_in_camera[3]:
                    # number of zeros in the array
                    num_zeros = np.count_nonzero(_colour_masks[i])
                    # counts the number of non zeros in the array, checks whether its above 1000  
                    if num_zeros > 3000:
                        # check if some of the colour is in the image and the colour isnt in false and there is not colour found
                        if moment['m00'] > 0 and self.colours_in_camera[i] == False and not self.colours_found[i]:
                            # checks the bottum to 275 height up of the _mask 
                            if (_colour_masks[i])[290:height, 0:width].any():
                                # ends the roation because a colour was found
                                self.end_rotation = True
                                # found an object in screen
                                self.object_found = True
                                # sets the colours being looked at in the index
                                self.current_index = i        
                                # sets the colour in the camera to TRUE
                                self.colours_in_camera[self.current_index] = True
                                print("found colour: ",self.current_index) 
                # checks if the colour is in camera and hasnt been found 
                elif self.colours_in_camera[i] and not self.colours_found[self.current_index]:
                    # checks to see if colour is in the image
                    if moment['m00'] > 0:               
                        # ends the roations so below can assign coordinates or find the colour                    
                        self.end_rotation = True
            
            # when rotaion is TRUE search for colours within a meter
            if self.end_rotation: 
                # RED, GREEN, BLUE, YELLOW order
                # loops through each colour mask  
                # the colour hasnt been found
                if self.colours_in_camera[self.current_index] == True and not self.colours_found[self.current_index]:
                    # if it finds the colour
                    # All the features of the single mask
                    moment = cv2.moments(_colour_masks[self.current_index])
                    # checks if colour is still in the image and 
                    if moment['m00'] > 0:
                        #print("colours in screen")
                        # Determine the distance of the object from the centre of the screen
                        # in order to correct the path of the robot if it's off course
                        cx = int(moment['m10']/moment['m00'])
                        cy = int(moment['m01']/moment['m00'])
                        # draws the circle around the image and hsv image
                        cv2.circle(self.bigMask, (cx, cy), 30, (0,0,255), -1)
                        cv2.circle(self.image, (cx, cy), 20, (0,0,255), -1)
                        # used for finding the centroid 
                        err = cx - width/2
                        # checks to see if the movement is 0 
                        # print("Adjusting robot z: ",self.twist.angular.z)
                        # print("Range from colour: ", self.rangeOfObstacle)
                        if (-float(err) / 100) != 0.000:
                            # while not true twist the error to make straight head on centroid 
                            self.twist.angular.z = -float(err) / 100
                            # publish to twist
                            self.publish_teleop.publish(self.twist)
                            print(self.rangeOfObstacle)
                        elif not math.isnan(float(self.rangeOfObstacle)):
                            self.twist.angular.z = 0.0    
                            self.publish_teleop.publish(self.twist)
                            # checks to see if its within a meter                        
                            if self.rangeOfObstacle <= 1:
                                print("1 METER AWAY")
                                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                                print("found the colour; index: ", self.current_index)
                                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                                # marks the colour of the current colour its on 
                                self.found_colour = True
                                # sets found colour to TRUE
                                self.colours_found[self.current_index] = True
                                self.colours_in_camera = [False, False, False, False]
                            # else if its greater than 1 but not nan create waypoints 
                            elif not self.waypoints_set:
                                # print("Creating waypoints for colour")
                                # create coordinates for around the colour found 
                                self.create_coordinated_to_colour()
                                # set found colour to true so we can move to waypoints around colour
                                self.search_camera = False
                                # waypoints around the colour have been set                                
                                self.waypoints_set = True
                        else:
                             
                            self.colours_in_camera = [False, False, False, False]      
        
        #---------------Displaying Text on image------------------
        #---------------------------------------------------------
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Calculations for putting timer on the screen         
        if not self.complete:
            self.seconds_timer = (time.time() - self.start_time)
            self.seconds = self.seconds_timer
            if self.seconds_timer >= 59:            
                self.start_time = time.time()
                self.minutes += 1
            minutes = str(self.minutes)
            seconds = str(int(math.floor(self.seconds))) 
            
            if int(len(str(abs(self.minutes)))) == 1:
                minutes = "0"+str(self.minutes)
            if int(len(str(abs(int(math.floor(self.seconds)))))) == 1:           
                seconds = "0"+str(int(math.floor(self.seconds)))
            self.timer_label = ("TIMER: "+minutes+":"+seconds+":"+str(float("{0:.2f}".format(self.seconds)))[-2:])
       
        objective_funtion_label = ""
        
        if self.search_camera:
            objective_funtion_label = "Current Objective: Searching for colours"
        else:
            objective_funtion_label = "Current Objective: Moving to Waypoint"
            
        current_waypoint_label = "Traveling to coordinate: "+"{0:.2f}".format(float(self.current_waypoint[0]))+","+"{0:.2f}".format(float(self.current_waypoint[1]))
        
        colour_found_list_label =[["RED: "+ str(self.colours_found[0])],
                                  ["GREEN: "+str(self.colours_found[1])],
                                  ["BLUE: "+str(self.colours_found[2])],
                                  ["YELLOW: "+str(self.colours_found[3])]]
                                  
        cv2.putText(self.image, self.timer_label, (10,350), cv2.FONT_HERSHEY_PLAIN, 1.5, (255,255,255),2,cv2.LINE_AA)
        
        cv2.putText(self.image, str(colour_found_list_label[0]), (10,375), cv2.FONT_HERSHEY_PLAIN, 1.5,(0,0,255), 2,cv2.LINE_AA)
        cv2.putText(self.image, str(colour_found_list_label[2]), (10,400), cv2.FONT_HERSHEY_PLAIN, 1.5, (255,0,0), 2,cv2.LINE_AA)
        cv2.putText(self.image, str(colour_found_list_label[1]), (200,375), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2,cv2.LINE_AA)
        cv2.putText(self.image, str(colour_found_list_label[3]), (200,400), cv2.FONT_HERSHEY_PLAIN, 1.5, (128,0,128), 2,cv2.LINE_AA)
        
        cv2.putText(self.image, objective_funtion_label, (10,425), cv2.FONT_HERSHEY_PLAIN, 1.5, (255,255,255),2,cv2.LINE_AA)
        cv2.putText(self.image, current_waypoint_label, (10,450), cv2.FONT_HERSHEY_PLAIN, 1.5, (255,255,255),2,cv2.LINE_AA)
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        #---------------------------------------------------------
        #---------------------------------------------------------
        
        # Show the mask and image
        cv2.imshow("Camera Veiw:", self.image)
        cv2.waitKey(1)

        
    # Define function to close the waypoint once the robot has reached it
    def xy_current_positions(self, data):
         # Take data from move_base/feedback to find the current position of the
         # robot
         self.robotposx = data.pose.pose.position.x
         self.robotposy = data.pose.pose.position.y
         
         # reads the orientation of the robot to work out the angle
         orientation_q = data.pose.pose.orientation
         orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
         (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
         
         yaw_degrees = yaw * 180 / math.pi
         self.radian_angle = yaw         
         if yaw_degrees< 0:
             yaw_degrees = yaw_degrees + 360
         
         self.orientation_z = yaw_degrees
        
    
    # creates four coordinates around the colour cylinder for error handling 
    def create_coordinated_to_colour(self):
        print("CREATING COORDINATES")
        print("----")
        # prints the robots current position
        print("current robots x position: ",self.robotposx)
        print("current robots y position: ",self.robotposy)
        print("----")
        print("TEST robots x position: ",self.Potential_X_Robit)
        print("TEST robots y position: ",self.Potential_Y_Robit)
        print("----")

        # stores the x and y of the colour cyclinder found 
        x = 0.000
        y = 0.000

        self.created_coordinates = True  
        if self.orientation_z < 90 and self.orientation_z >= 0:
            angle = self.orientation_z - 0

            #ACTUAL COLOUR LOCATION POSITIVE
            # x for found colour 
            x = (self.robotposx + (math.cos(math.radians(angle)) * self.rangeOfObstacle)) 
            # y for found colour 
            y  = (self.robotposy + (math.sin(math.radians(angle)) * self.rangeOfObstacle))
            
        elif self.orientation_z < 180 and self.orientation_z >= 90:
            angle = self.orientation_z - 90
            
            #ACTUAL COLOUR LOCATION POSITIVE
            # x
            x = (self.robotposx - (math.sin(math.radians(angle)) * self.rangeOfObstacle))
            # y            
            y = self.robotposy + (math.cos(math.radians(angle)) * self.rangeOfObstacle)

        elif self.orientation_z < 270 and self.orientation_z >= 180:
            angle = self.orientation_z - 180

            #ACTUAL COLOUR LOCATION POSITIVE
            # x
            x = (self.robotposx - (math.cos(math.radians(angle)) * self.rangeOfObstacle))
            # y            
            y = (self.robotposy - (math.sin(math.radians(angle)) * self.rangeOfObstacle))
            
        elif self.orientation_z < 360 and self.orientation_z >= 270:
            angle = self.orientation_z - 270

            #ACTUAL COLOUR LOCATION POSITIVE
            # x
            x = (self.robotposx + (math.sin(math.radians(angle)) * self.rangeOfObstacle))
            # y            
            y =  (self.robotposy - (math.cos(math.radians(angle)) * self.rangeOfObstacle)) 
        
        # ABOVE, LEFT, BELOW, RIGHT
        # ABOVE
        self.coordinated_around_colour[0][0] = x
        self.coordinated_around_colour[0][1] = y + 0.55
        # LEFT
        self.coordinated_around_colour[1][0] = x - 0.55
        self.coordinated_around_colour[1][1] = y
        # BELOW
        self.coordinated_around_colour[2][0] = x
        self.coordinated_around_colour[2][1] = y - 0.55
        # RIGHT
        self.coordinated_around_colour[3][0] = x + 0.55
        self.coordinated_around_colour[3][1] = y 
         
        # prints the orientates of 
        print("Angle: ",self.orientation_z)
        print("----")
        print("Distance from current positon: ", self.rangeOfObstacle)
        print("----")
        print("X position of colour: ", x)     
        print("y position of colour: ", y) 
        print("----")
        print("Above X: ", self.coordinated_around_colour[0][0])
        print("Above Y: ", self.coordinated_around_colour[0][1])
        print("----")
        print("Left X: ", self.coordinated_around_colour[1][0])
        print("Left Y: ", self.coordinated_around_colour[1][1])
        print("----")
        print("Below X: ", self.coordinated_around_colour[2][0])
        print("Below Y: ", self.coordinated_around_colour[2][1])
        print("----")
        print("Right X: ", self.coordinated_around_colour[3][0])
        print("Right Y: ", self.coordinated_around_colour[3][1])
        print("----")
        
    
    # creates waypoints around the hard core waypoints to stop object collision
    def waypoint_calculator(self, goal):
        # assigns the x and y of goal to x and y variables 
        x = goal[0]
        y = goal[1]
        
        # ABOVE, LEFT, BELOW, RIGHT
        # ABOVE
        self.coordinate_around_original_waypoint[0][0] = x
        self.coordinate_around_original_waypoint[0][1] = y + 1
        # LEFT
        self.coordinate_around_original_waypoint[1][0] = x - 1
        self.coordinate_around_original_waypoint[1][1] = y
        # BELOW
        self.coordinate_around_original_waypoint[2][0] = x
        self.coordinate_around_original_waypoint[2][1] = y -1
        # RIGHT
        self.coordinate_around_original_waypoint[3][0] = x + 1
        self.coordinate_around_original_waypoint[3][1] = y 
        
        
    # gets the laser scan of the object infront of the robot 
    def laser_call(self, data):
        global distance
        # Limit Laser
        data.angle_min = -1
        data.angle_max = 1      
        distance = min(data.ranges)
        # Gets the object head on
        self.rangeOfObstacle = data.ranges[len(data.ranges)/2]

    
    def rotate(self):
         # Pie
        PI = 3.1415926535897
        #Starts a new node
        #rospy.init_node('robot_cleaner', anonymous=True)
        #velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
    
        # Receiveing the user's input
        print("Let's rotate your robot")
    
        #Converting from angles to radians
        angular_speed = 1
        relative_angle = 360*2*PI/360

    
        # Checking if our movement is CW or CCW
        self.twist.angular.z = -abs(angular_speed)
    
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle) and not self.end_rotation:
            self.publish_teleop.publish(self.twist)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
    
    
        #Forcing our robot to stop
        self.twist.angular.z = 0
        self.publish_teleop.publish(self.twist)
        
    
    # MOVE_BASE STACK
    # MOVE_BASE STACK
    # MOVE_BASE STACK - move to waypoint
    def _move_to_waypoint(self, waypoint):
        
        # assigns the move_base action lib moving vairbale 
        # sets the waypoints [0] [1] for X and Y
        Move_to = Pose(Point(waypoint[0], waypoint[1], 0.000), Quaternion(0.000, 0.000, 0.000, 1))
        
        # Subscribe to the move_base action server
        move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction) 
        
        # initialise the wait for server before sending the waypoint
        while(not move_base.wait_for_server(rospy.Duration.from_sec(5.0))):
            print("Waiting for move_base action server...")
     
        # sets up the movebasegoal used for senting waypoints
        goal = MoveBaseGoal()
        # Set up the next goal location
        goal.target_pose.header.frame_id = 'map'
        # sets the stamp to current time
        goal.target_pose.header.stamp = rospy.Time.now()
        # set the position to move to 
        goal.target_pose.pose = Move_to
        # Start the robot toward the next location
        move_base.send_goal(goal, self._done_cb, self._active_cb)
        # holds the move_base whilst its status us checked
        move_base.wait_for_result()        


    # MOVE_BASE STACK - assigns activity 
    def _active_cb(self):
        print("Goal pose is now being processed by the Action Server...")
        print("----")

    # MOVE_BASE - checks status 
    def _done_cb(self, status, result):
        
        # individual status check; provides print to console for debugging
        if status == 2:
            print("Goal pose received a cancel request after it started executing, completed execution!")
            print("----")
        if status == 3:
            print("Goal pose reached") 
            print("----")
        if status == 4:
            print("Goal pose was aborted by the Action Server")
            print("----")
            return
        if status == 5:
            print("Goal pose has been rejected by the Action Server")
            print("----")
            return
        if status == 8:
            print("Goal pose received a cancel request before it started executing, successfully cancelled!")
            print("----")
            
    
    # checks to see whether the assignment has been completed
    def checkComplete(self):
        if(self.colours_found[0] and self.colours_found[1] and self.colours_found[2] and self.colours_found[3]):
            return True
        else:
            return False

    
    # Main method used to control the class 
    def _main(self):
        self.start_time = time.time()
        print("Starting Object Finder Assignment")
        # iterate through all the goals 
        for goal in self.map_cordinates:
            # stores the current goal 
            self.current_waypoint = goal 
            print("Running the GOAl")
            print("----")
            print("----")
            print("Found Colour Red: ",self.colours_found[0])  
            print("Found Colour Green: ",self.colours_found[1])
            print("Found Colour Blue: ",self.colours_found[2])
            print("Found Colour Yellow: ",self.colours_found[3])
            print("----")
            print("----")
            # checks if assignment has been completed (found all colours)
            if not self.checkComplete():
                print("Not Completed YET")
                print("moving to this waypoint: ", goal) 
                print("----")
                # stops searching whilst moving to way point
                self.search_camera = False
                # moving to each iteration of waypoint
                self._move_to_waypoint(goal)
                #sets the rotation to FALSE for while loop 
                self.end_rotation = False
                # loops back to current waypoint when an object is found to complete scanning 
                while not self.end_rotation:
                    # setting the set waypoints to FALSE to allows more to be created when a colour is found
                    self.waypoints_set = False
                    # object in the view of camera is set to FALSE, ready to find colours
                    self.object_found = False
                    # FALSE to allow colour to found when searching 
                    self.found_colour = False
                    # start searching for colours as the robot rotates 
                    self.search_camera = True
                    # rotate the robot 360 to look for colours 
                    self.rotate()
                    # Wait for 2 seconds to align robot to centroid of shape 
                    time.sleep(1)
                    # checks if a cylinder was seen and that cylinder hasnt already been found 
                    if self.object_found and not self.colours_found[self.current_index]:
                        print("This colour was found in camera view: ", self.current_index)
                        # loop through waypoints around the colour to try and find the colour
                        for colour_waypoint in self.coordinated_around_colour:
                            # colour NOT found yet
                            if not self.found_colour:
                                self.current_waypoint = colour_waypoint
                                print("Trying to find colour cylinder at: ", colour_waypoint)
                                print("----")
                                # stop searching for colours whilst moving
                                self.search_camera = False
                                # move to waypoint around the colour cylinder
                                self._move_to_waypoint(colour_waypoint)
                                # set for rotation 
                                self.end_rotation = False
                                # allows the if statement to break if a colour wasnt found                                 
                                self.object_found = False
                                # allow searching for the colour
                                self.search_camera = True
                                # start rotating 
                                self.rotate()
                                # Wait for 2 seconds
                                time.sleep(2)
                            else:
                                self.end_rotation = False
                                
                        # stop searching whilst moving
                        self.search_camera = False
                        # after chekcing all the waypoint around the colour go back to programmed waypoint
                        self.current_waypoint = goal
                        self._move_to_waypoint(goal)
                    # if no object colour found in camera view or colour was found already 
                    else:  
                        # end rotaion while loop 
                        self.end_rotation = True
                        # stop searching whilst moving
                        self.search_camera = False
                        print("Setting all the colours in camera to FALSE")
                        print("----")
                        # sets all the colours in the camera to False
                        self.colours_in_camera = [False, False, False, False] 
        
        # print the results after all goals have been too or the program ended because all colours where found                                                      
        print("----")
        print("----")
        print("Found Colour Red: ",self.colours_found[0])  
        print("Found Colour Green: ",self.colours_found[1])
        print("Found Colour Blue: ",self.colours_found[2])
        print("Found Colour Yellow: ",self.colours_found[3])
        print("----")
        print("----")        
        print("Finished Object Finder Assignment")
        
# Python .py files Main method call
def main():
    rospy.init_node('robotsAssignment', anonymous=True)
    AutonomouseRoboticsAssignment()
    rospy.spin()
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()
    
          
