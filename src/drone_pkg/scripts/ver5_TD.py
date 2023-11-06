#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
import sys
import pygame

from utils import ARUCO_DICT, aruco_display

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOLRequest, CommandTOL
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

current_state = State()     # PX4 autopilot state (armed, offboard, etc)
pose = PositionTarget()
car_pose = Twist()

pose.coordinate_frame = 8   # Setting the coordinate frame to body ned, to control the drone from body frame
pose.type_mask = 1024   # To be able to yaw the drone, we must do this. In default setting, yaw rate doesnt work 

# Controller gains
Kp = 0.006    # tuned value = 0.006
Ki = 0      # no integral term in alignment of drone with stationary car
Kd = 0.18   # tuned value = 0.18

# Kpy = 0     # Since the frame height != width, by giving different gains to remove x and y errors,both errors would become 0 at the same time
# Kiy = 0    
# Kdy = 0
error_integral_x = 0    # For integral term of controller
error_integral_y = 0

# def aruco_pose(frame,ids,corners,matrix_coefficients, distortion_coefficients): # For position of aruco marker wrt camera

#     ids = ids.flatten()
#     if len(corners) > 0:
#         for i in range(0, len(ids)):
#             # Estimate pose of each marker and return the values rvec and tvec, 0.1 is the size of the marker in meters
#             rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.1, matrix_coefficients, distortion_coefficients)
#     # Draw a square around the markers 
#     # cv2.aruco.drawDetectedMarkers(frame, corners) 

#             # Draw Axis
#             # cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01) 
        
#     return frame, rvec, tvec    # tvec: Translation vector

# def aruco_detect(frame):

#     matrix_coefficients= np.array([[454.6857718666893, 0.0, 424.5], [0.0, 454.6857718666893, 240.5], [0.0, 0.0, 1.0]])
#     distortion_coefficients= np.array([0.0, 0.0, 0.0, 0.0, 0.0])    # Obtained from camera info topic
    
#     marker_dictionary = {}  # To store data of the detected markers

#     arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_ARUCO_ORIGINAL"])
#     arucoParams = cv2.aruco.DetectorParameters_create()

#     if ARUCO_DICT.get("DICT_ARUCO_ORIGINAL", None) is None:
#         print("ArUCo tag type DICT_ARUCO_ORIGINAL is not supported")
#         sys.exit(0)

#     corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

#     if len(corners) > 0:

#         # Aruco pose estimation
#         frame, rvec, tvec = aruco_pose(frame,ids,corners,matrix_coefficients, distortion_coefficients)
    
#         # flatten the ArUco IDs list
#         ids = ids.flatten()
        
#         # i is to control the index for marker_dictionary array 
#         i=0
        
#         # loop over the detected ArUCo corners
#         for (markerCorner, markerID) in zip(corners, ids):
#         # extract the marker corners (which are always returned
#         # in top-left, top-right, bottom-right, and bottom-left
#         # order)

#             corners = markerCorner.reshape((4, 2))
#             (topLeft, topRight, bottomRight, bottomLeft) = corners

#             # convert each of the (x, y)-coordinate pairs to integers as they are in float form, and are pixel coordinates
#             topRight = (int(topRight[0]), int(topRight[1]))
#             bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
#             bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
#             topLeft = (int(topLeft[0]), int(topLeft[1]))

#             # draw the bounding box of the ArUCo detection
#             cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
#             cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
#             cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
#             cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
#             cv2.circle(frame,topLeft, 6, (0,0,255), -1)

#             # compute and draw the center (x, y)-coordinates of the
#             # ArUco marker
#             cX = int((topLeft[0] + bottomRight[0]) / 2.0)
#             cY = int((topLeft[1] + bottomRight[1]) / 2.0)

#             # Direction of motion; put if condition for tags representing robots
#             magnitude = ((topLeft[0]-bottomLeft[0])**2 + (topLeft[1]-bottomLeft[1])**2)**0.5
#             # Unit vectors pointing towards direction of robot motion
#             wX = (topLeft[0]-bottomLeft[0])/ magnitude
#             wY = (topLeft[1]-bottomLeft[1]) / magnitude
#             theta = 180*np.arccos(wX)/3.14159

#             if (wY < 0):
#                 theta = 360 - theta

#             # Directions UP, LEFT, DOWN and RIGHT are represented by numbers 8, 4, 2, 6 respectively
#             if theta >= 45 and theta < 135:
#                 direction = 2
#             elif theta >= 135 and theta < 225 :
#                 direction = 4
#             elif theta >= 225 and theta < 315:
#                 direction = 8
#             if (theta >= 0 and theta < 45) or (theta > 315 and theta < 360):
#                 direction = 6

#             #Dictionary to store all the required information about one robot
#             marker_dictionary[i] = {"id": markerID, "cX" : cX,"cY" : cY, "direction" : direction, "rotation_vector": rvec, "translation_vector": tvec }
#             i = i+1
            
#             # print(marker_dictionary)

#             # draw the ArUco marker ID on the frame
#             cv2.putText(frame, str(markerID),
#                 (topLeft[0], topLeft[1] - 15),
#                 cv2.FONT_HERSHEY_SIMPLEX,
#                 0.5, (0, 255, 0), 2)

#     return frame, marker_dictionary

def state_cb(msg):  # Eg: Offboard, armed, etc; the live feed of the state is updated in the msg variable
    global current_state
    current_state = msg

class video_feed:   # Class used to represent the video feed

    def __init__(self): # __init__ method contains a collection of statements(i.e. instructions) that are executed at the time of Object creation. It runs as soon as an object of a class is instantiated
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, callback = self.callback)
        self.error_list = 0   # Initializing the error list to be returned into main function with a random value = 0
    
    def callback(self, image):
        
        # print("Callback entered")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape   # rows = 480, cols = 848 in gazebo camera
        # print(rows,cols)

        target_pointX = cols/2
        target_pointY = rows/2

        # cv_image , marker_dictionary = aruco_detect(cv_image)

        # print(marker_dictionary)

        # if marker_dictionary != {}:

        #     errorX = (target_pointX - marker_dictionary[0]["cX"]) # / cols  # Dividing by cols to normalize the error so that it can be used for any aspect ratio camera
        #     errorY = (target_pointY - marker_dictionary[0]["cY"]) # / rows       # Error between centre of square target and centre of frame, assuming there is only 1 target here
            
        #     self.error_list = [errorX, errorY, marker_dictionary[0]['cX'], marker_dictionary[0]["cY"]]

        # cv2.imshow("Detection Status", cv_image)
        # cv2.waitKey(3)

        # Below this is an alternative thresholding algorithm for landing pad detection
        
        # print("Centre of frame ", (target_pointX, target_pointY))

        grayscale_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)    # Converting image to gray scale

        # cv2.imshow("Grayscale", grayscale_image)   # Grayscale only contains luminance (brightness) information and no color information; 
                                        # that is why maximum luminance is white and zero luminance is black; everything in between is a shade of gray

        ret,thresh = cv2.threshold(grayscale_image,200,255,0)   # Thresholding to get a binary image; input has to be a grayscale image. 2nd argument is the threshold value; 255 = white

        contours,hierarchy = cv2.findContours(thresh, 1, 2)     # Detect all contours in the image
        # print("Number of contours detected:", len(contours))

        for cnt in contours:
            x1,y1 = cnt[0][0]   # Left corner pixel coordinates
            # print((x1,y1))
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(cnt)
                centre_x = x + w/2
                centre_y = y + h/2

                # print("centre of rectangle ", (centre_x, centre_y))
                cv_image = cv2.drawContours(cv_image, [cnt], -1, (0,255,255), 3)
                # cv2.putText(output_image, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.circle(cv_image, (int(centre_x),int(centre_y)), radius=10, color=(100, 100, 100), thickness = -1)

                errorX = (target_pointX - centre_x) # / cols  # Dividing by cols to normalize the error so that it can be used for any aspect ratio camera
                errorY = (target_pointY - centre_y) # / rows       # Error between centre of square target and centre of frame, assuming there is only 1 target here

                # print("You can autonomously follow the detected rectangle now")

                self.error_list = [errorX, errorY, centre_x, centre_y]
            
        cv2.imshow("Detection Status", cv_image)
        cv2.waitKey(3)

    def get_data(self):
        return self.error_list
            
def set_mode(mode):
    rospy.wait_for_service('mavros/set_mode')  # Waiting until the service starts
    offb_set_mode.custom_mode = mode 
    for i in range(100):
        if(rospy.is_shutdown):
            break
        local_pos_pub.publish(pose)
        rate.sleep()
    try:
        # setmission = rospy.ServiceProxy('mavros/set_mode', SetMode) # Creating a proxy service for the rosservice named /mavros/set_mode for entering offboard mode
        set_mode_client(offb_set_mode)
        rospy.loginfo("Vehicle's current mode: " + str(offb_set_mode.custom_mode))
        # print("The current state is\n",current_state)
        # print('2', setmission(220, "OFFBOARD"))

    except rospy.ServiceException as e:
        print ("Service set_mode call failed: %s"%e)

def setArm(val):    # To set the drone to state armed/ disarmed
    arm_cmd.value = val
    # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure 
    rospy.wait_for_service('mavros/cmd/arming')  # Waiting until the service starts
    try:
        #armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
        arming_client(arm_cmd) # CommandBool variables take only True or False values 
        rospy.loginfo("Vehicle arming state: " + str(arm_cmd.value))
        # print("The current state is\n",current_state)

    except rospy.ServiceException as e:
        print ("Service setarm call failed: %s"%e)

# def disArm():
#     # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
#     # rospy.wait_for_service('mavros/cmd/arming')  # Waiting until the service starts 
#     try:
#         arming_client(False)
#         rospy.loginfo("Vehicle disarmed")
#         print("The current state is",current_state)

#     except rospy.ServiceException as e:
#         print ("Service disarming call failed: %s"%e)

def Kp_value(val):
    global Kp
    Kp = val.data

def Kd_value(val):
    global Kd
    Kd = val.data

def Ki_value(val):
    global Ki
    Ki = val.data   # As the message type is std_msgs Float32 class, containing data in float datatype 

if __name__ == "__main__":

    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

    car_pub = rospy.Publisher("vehicle_blue/cmd_vel", Twist, queue_size = 10)

    # rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    # rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # rospy.wait_for_service("/mavros/set_mode")
    # takeoff_client = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

    Kp_sub = rospy.Subscriber("/Kp", Float32, callback = Kp_value)
    Ki_sub = rospy.Subscriber("/Ki", Float32, callback = Ki_value)
    Kd_sub = rospy.Subscriber("/Kd", Float32, callback = Kd_value)

    from_video = video_feed()   # Variable representing the class video_feed

    # Setpoint publishing MUST be faster than 20Hz
    rate = rospy.Rate(20)
    
    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # initialising pygame
    pygame.init()
    
    # creating display
    display = pygame.display.set_mode((100, 100))

    offb_set_mode = SetModeRequest()    # Format of argument in the service call to change mode to offboard
    # offb_set_mode.custom_mode = 'OFFBOARD'
    arm_cmd = CommandBoolRequest()  # Format of argument in the service call to change state to armed
    # arm_cmd.value = True
    
    # last_req = rospy.Time.now()
    key = 0
    auto_mode = 0

    while not rospy.is_shutdown():

        error_list = from_video.get_data()

        # if error_list != 0:
            # print("Press u to enter autonomous mode to follow the rectangle")
            # print("Error list", error_list)

        # if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
        #     print("enabling offboard mode")
        #     if(set_mode_client.call(offb_set_mode).mode_sent == True):
        #         rospy.loginfo("OFFBOARD enabled")
            
        #     last_req = rospy.Time.now()
        # else:
        #     if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
        #         if(arming_client.call(arm_cmd).success == True):
        #             rospy.loginfo("Vehicle armed")
            
        #         last_req = rospy.Time.now()

        if key ==1:
            if auto_mode == 0:
                local_pos_pub.publish(pose)     # If key = 1, non zero velocity commands are published
            car_pub.publish(car_pose)   # If key = 1, publish the car position commands as well

        elif key ==0:
            if auto_mode == 0: 
                pose.velocity.x = 0
                pose.velocity.y = 0
                pose.velocity.z = 0
                local_pos_pub.publish(pose)
                pose.yaw_rate = 0
            car_pose.linear.x = 0
            car_pose.angular.z = 0
            car_pub.publish(car_pose)

        for event in pygame.event.get():    # Events refer to the actions a user performs, eg: Clicking a key on the keyboard/ mouse. They are stored in an event queue (FIFO)
            # print(event)
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                key = 1

                if event.key == pygame.K_w:
                    pose.velocity.x = 1
                    # print("pitch forward at "+ str(pose.velocity.x) + "m/s")  
                
                elif event.key == pygame.K_s:
                    pose.velocity.x = -1
                    # print("pitch back at "+ str(pose.velocity.x) + "m/s")

                elif event.key == pygame.K_a:
                    pose.velocity.y = 1
                    # print("roll left at "+ str(pose.velocity.y) + "m/s")

                elif event.key == pygame.K_d:
                    pose.velocity.y = -1
                    # print("roll right at "+ str(pose.velocity.y) + "m/s")

                elif event.key == pygame.K_z:
                    pose.velocity.z = 3
                    # print("height increase at "+ str(pose.velocity.z) + "m/s")         

                elif event.key == pygame.K_x:
                    pose.velocity.z = -1 
                    # print("height decrease at "+ str(pose.velocity.z) + "m/s")

                elif event.key == pygame.K_q:
                    pose.yaw_rate = 1      # 1 radian per sec
                    # print("yaw positive z at - "+ str(pose.yaw) + "rad/s")   # YAW RATE NOT WORKING, ITS ROTATING TO A FIXED ANGLE
                
                elif event.key == pygame.K_e:
                    pose.yaw_rate = -1
                    # print("yaw negative z at - "+ str(pose.yaw) + "rad/s")

                elif event.key == pygame.K_l:
                #     # if (offb_set_mode.custom_mode != 'AUTO.LAND'):
                #     #     offb_set_mode.custom_mode = 'AUTO.LAND'
                #     # if(current_state.mode != "AUTO.LAND"):
                #     #     if(set_mode_client.call(offb_set_mode).mode_sent == True):
                #     #         rospy.loginfo(current_state.mode + "enabled")
                #     # arm_cmd.value = False
                #     # if(not current_state.armed):
                #     #     if(arming_client.call(arm_cmd).success == True):
                #     #             rospy.loginfo("Vehicle armed")
                    set_mode("AUTO.LAND")
                    # rospy.loginfo(current_state.mode +  " enabled")
                    # print("The current state is\n", current_state)
                    # setArm(False)    # Disarm after landing
                    # set_mode("POSCTL")

                elif event.key == pygame.K_o:   # Send to offboard mode by pressing o
                    # offb_set_mode.custom_mode = 'OFFBOARD'
                    # if(current_state.mode != "OFFBOARD"):
                    #     if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    #         rospy.loginfo( current_state.mode + " enabled")

                    set_mode("OFFBOARD")
                    # rospy.loginfo( current_state.mode + " enabled")
                    # arm_cmd = CommandBoolRequest()
                    # arm_cmd.value = True
                    # if(not current_state.armed):
                    #     if(arming_client.call(arm_cmd).success == True):
                    #         rospy.loginfo("Vehicle armed")
                
                elif event.key == pygame.K_n:   # To arm the drone
                    setArm(True)

                elif event.key == pygame.K_m:   # To disarm the drone
                    setArm(False) 

                elif event.key == pygame.K_u:
                    x_array = [0]
                    time_spent = [0]    # Start with [0] to prevent index error
                    y_array = [0]
                    start_time = time.time()    # Time at which autonomous mode starts
                    initial_error = from_video.get_data()
                    auto_mode = 1   # Autonomous mode enabled
                    input_x = [0]
                    input_y = [0]

                    Px_array = [0]
                    Ix_array = [0]
                    Dx_array = [0]
                    cmd_array = [0]

                    Py_array = [0]
                    Iy_array = [0]
                    Dy_array = [0]

                elif event.key == pygame.K_i:

                    auto_mode = 0   # Auto mode disabled

                    # Initialise the subplot function using number of rows and columns
                    figure, axis = plt.subplots()

                    # axis.plot(np.array(time_spent), np.array(x_array))   # Plotting pixel error vs time
                    # axis.set_xlabel("time")  # add X-axis label
                    # axis.set_ylabel("X- error")  # add Y-axis label
                    # axis.set_title("X-axis error vs time")  # add title

                    # axis[0,1].plot(np.array(time_spent), np.array(Px_array))   
                    # axis[0,1].set_xlabel("time")  # add X-axis label
                    # axis[0,1].set_ylabel("Proportional term - X")  # add Y-axis label
                    # axis[0,1].set_title("Proportional component - X vs time")  # add title

                    axis.plot(np.array(time_spent), np.array(y_array))   
                    axis.set_xlabel("time")  # add X-axis label
                    axis.set_ylabel("Y- axis error")  # add Y-axis label
                    axis.set_title("Y- axis error vs time")  # add title

                    # axis[1,0].plot(np.array(time_spent), np.array(Py_array))  
                    # axis[1,0].set_xlabel("time")  # add X-axis label
                    # axis[1,0].set_ylabel("Proportional term - Y")  # add Y-axis label
                    # axis[1,0].set_title("Proportional component - Y vs time")  # add title

                    # # axis[2,0].plot(np.array(time_spent), np.array(cmd_array))   
                    # # axis[2,0].set_xlabel("time")  # add X-axis label
                    # # axis[2,0].set_ylabel("velocity command")  # add Y-axis label
                    # # axis[2,0].set_title("velocity command vs time")  # add title

                    plt.show()

                    error_integral_x = 0
                    error_integral_y = 0
                    x_array = [0]
                    y_array = [0]
                    Px_array = [0]
                    Ix_array = [0]
                    Dx_array = [0]
                    Py_array = [0]
                    Iy_array = [0]
                    Dy_array = [0]
                    cmd_array = [0]
                    input_x = [0]
                    input_y = [0]

                    # set_mode("AUTO.LAND")
                    # rospy.loginfo(current_state.mode +  " enabled")
                    # print("The current state is\n", current_state)
                    # setArm(False)

                elif event.key == pygame.K_UP:  # Up arrow key
                    car_pose.linear.x = 1
                    # print("differential drive robot moved forward at " + str(car_pose.linear.x) + "m/s")

                elif event.key == pygame.K_DOWN:
                    car_pose.linear.x = -1
                    # print("differential drive robot moved backward at " + str(car_pose.linear.x) + "m/s")
                
                elif event.key == pygame.K_RIGHT:
                    car_pose.angular.z = -1
                    # print("differential drive robot rotated anticlockwise at " + str(car_pose.angular.x) + "m/s")
                
                elif event.key == pygame.K_LEFT:
                    car_pose.angular.z = 1
                    # print("differential drive robot rotated clockwise at " + str(car_pose.angular.x) + "m/s")                 

            if event.type == pygame.KEYUP:  # If key is released, drone stays in that location
                key = 0
                                                
        if auto_mode == 1:

            error_list = from_video.get_data()  # Taking the updated error list every iteration
            print(error_list)

            if abs(error_list[0]) > 5 or abs(error_list[1]) > 5:

                x_array.append(error_list[0])
                y_array.append(error_list[1])
                input_x.append(error_list[3])
                input_y.append(error_list[2])
                time_spent.append(time.time() - start_time)

                # dispalacement_vector_car = [error_list[2] - initial_error[2] ,  error_list[3] - initial_error[3]]       # The relative displacement of the car with respect to the camera frame
                # velocity_vector_car = [dispalacement_vector_car[i] / (time_spent[-1] - time_spent[-2]) for i in range(2)]  # Relative velocity with respect to camera frame

                initial_error = error_list

                error_integral_x += error_list[1] * (time_spent[-1] - time_spent[-2])
                error_integral_y += error_list[0] * (time_spent[-1] - time_spent[-2])

                # error_integral_x += error_list[1] * rate
                # error_integral_y += error_list[0] * rate

                Px = Kp*error_list[1]
                # Dx = Kd*(y_array[-1] - y_array[-2])*(time_spent[-1] - time_spent[-2])   # Controlling drone from drone frame, so x command on drone reduces y error in image frame and vice versa
                Dx = Kd*(-(input_x[-1] - input_x[-2]))*(time_spent[-1] - time_spent[-2])    # Derivative kickback
                Ix = Ki * error_integral_y

                Px_array.append(Px)
                Ix_array.append(Ix)
                Dx_array.append(Dx)
                cmd_array.append(Px + Ix + Dx)

                Py = Kp*error_list[0]
                # Dy = Kd*(x_array[-1] - x_array[-2])*(time_spent[-1] - time_spent[-2])
                Dy = Kd * ( - (input_y[-1] - input_y[-2]))*(time_spent[-1] - time_spent[-2])
                Iy = Ki * error_integral_x

                Py_array.append(Py)
                Iy_array.append(Iy)
                Dy_array.append(Dy)

                pose.velocity.x = Px + Ix + Dx    
                pose.velocity.y = Py + Iy + Dy
                local_pos_pub.publish(pose)

                # velocity_wrt_center = [-pose.velocity.x , -pose.velocity.y]     # Velocity of car towards center of frame due to pixel error (negative of drone command velocity)
                # velocity_w

            else:
                pose.velocity.x = 0    
                pose.velocity.y = 0
                local_pos_pub.publish(pose)
                # if current_state.mode != "AUTO.LAND":
                #     set_mode("AUTO.LAND")
                #     print (current_state.mode+  " activated")
                # disArm()
                # set_mode("POSCTL")              

        rate.sleep()
