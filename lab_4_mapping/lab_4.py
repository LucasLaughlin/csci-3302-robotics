import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import math
from math import cos, sin
from numpy import rot90
import os


# GLOBALS 
pose2d_sparki_odometry = Pose2D(0, 0, 0) #Pose2D message object, contains x,y,theta members in meters and radians
servo_angle = None
ping_distance = None
ir_sensor_readings = [0 for i in range(5)]
height = 24
width =36
max_dist = math.sqrt(width**2 + height**2)
map_coord = [0 for n in range(width*height)]
map_cell_size = 0.05
robot_map_index= 0
render_count = 0

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_= None
publisher_render_sim = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.1 # In seconds

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render_sim, publisher_odometry
    global IR_THRESHOLD, CYCLE_TIME, render_count
    global pose2d_sparki_odometry, ping_distance

    #TODO: Init your node to register it with the ROS core
    init()

    while not rospy.is_shutdown():
        start = time.time()
        msg = Float32MultiArray()
        #readSensors();
        if (ir_sensor_readings[2] < IR_THRESHOLD):
        
            #sparki.moveForward();
            msg.data = [1.0,1.0]
        
        elif (ir_sensor_readings[1] < IR_THRESHOLD) :
        
            msg.data = [-1.0,1.0]
         
        elif (ir_sensor_readings[3] < IR_THRESHOLD) :
        
            msg.data = [1.0,-1.0]

        else:

            msg.data = [-1.0,1.0]
        
        
        publisher_motor.publish(msg)
        publisher_ping.publish(Empty())
        render_count += 1
        
        if(render_count % 100000 == 0):
            publisher_render_sim.publish(Empty())
            os.system('cls' if os.name == 'nt' else 'clear')
            display_map()
        if False:
            rospy.loginfo("Loop Closure Triggered")
            
        if ((time.time()-start)<50):
            rospy.sleep(50 - time.time() - start)
        
        rospy.sleep(0)



def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render_sim, publisher_odometry
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry

    rospy.init_node('sparki', anonymous= True)

    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray) # Send two between [-1.0, 1.0] for left/right wheels
    publisher_ping = rospy.Publisher('/sparki/ping_command', Empty) # Tell sparki to ping ASAP
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16) # Tell Sparki to set servo motor to an angle between [-80, 80] degrees
    publisher_odometry = rospy.Publisher('/sparki/set_odometry', Pose2D) # Tell sparki to set odometry
    publisher_render_sim = rospy.Publisher('/sparki/render_sim',Empty)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state) # use json.loads to turn JSON dictionary to python object
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry) # contains x, y, theta

    rospy.sleep(0.2)
    publisher_servo.publish(45)
    pass

def callback_update_odometry(data):
    global pose2d_sparki_odometry
    pose2d_sparki_odometry.x = data.x
    pose2d_sparki_odometry.y = data.y
    pose2d_sparki_odometry.theta = data.theta
    robot_coord_to_map()
    pass
  
def callback_update_state(data):
    global servo_angle, ping_distance, ir_sensor_readings 
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic   
    servo_angle = state_dict['servo']
    ir_sensor_readings = state_dict['light_sensors']

    if (('ping' in state_dict) and (state_dict['ping']!=-1)):
        ping_distance = state_dict['ping']
        robot_coords = convert_ultrasonic_to_robot_coords(ping_distance)
        world_coords = convert_robot_coords_to_world(robot_coords[0], robot_coords[1])
        populate_map_from_ping(world_coords[0], world_coords[1])
    pass
  
# Given a ping distance, calculate x, y from robot frame of reference
def convert_ultrasonic_to_robot_coords(x_us):
    global pose2d_sparki_odometry, servo_angle
    x_r = x_us * cos(servo_angle)
    y_r = x_us * sin(servo_angle)
    return x_r, y_r

# Given coordinates in robot frame of reference, convert to world coordinates using homogenous transform 
def convert_robot_coords_to_world(x_r, y_r):
    global pose2d_sparki_odometry
    #Homogenouse transform derived from Transform matrix
    x_w = x_r * cos(pose2d_sparki_odometry.theta) - y_r * sin(pose2d_sparki_odometry.theta) + pose2d_sparki_odometry.x
    y_w = x_r * sin(pose2d_sparki_odometry.theta) + y_r * cos(pose2d_sparki_odometry.theta) + pose2d_sparki_odometry.y
    return x_w, y_w

# Given x, y coordinates, populate the map at that coordinate. 
# 0 is empty. 1 is occupied
def populate_map_from_ping(x_ping, y_ping):
    global map_cell_size, map_coord
    # divide(floor) the world coordinates by the cell size to arrive at cell coordiantes
    x_ping//=map_cell_size # i-coord
    y_ping//=map_cell_size # j-coord
    cell_index = ij_to_cell_index(int(y_ping),int(x_ping))
    if (cell_index<len(map_coord)):
        map_coord[cell_index] = 1
    pass

def robot_coord_to_map():
    global pose2d_sparki_odometry, robot_map_index
    r_map_coord_x=pose2d_sparki_odometry.x//map_cell_size
    r_map_coord_y=pose2d_sparki_odometry.y//map_cell_size
    robot_map_index = ij_to_cell_index(r_map_coord_y, r_map_coord_x)

#R is the robot position
#_ is an open spot
#X is a blocked spot
def display_map():
    global width, height, map_coord, robot_map_index, pose2d_sparki_odometry
    out = [["" for x in range(width)] for y in range(height)]
    for i in range(height):
        for j in range(width):
            if (map_coord[ij_to_cell_index(i,j)]== 0):
                out[i][j]="_"
            elif (map_coord[ij_to_cell_index(i,j)]== 1):
                out[i][j]="X"
            if (ij_to_cell_index(i,j)==robot_map_index):

                out[i][j]="R"
    print('\n'.join([''.join(['{:2}'.format(item) for item in (rowOut)]) for rowOut in reversed(out)]))
    print('\n\n')
    pass

# convert a x, y position in map table to a single value index of cell
def ij_to_cell_index(i,j):
    global width
    return (width * i + j)

# convert a cell index into an x, y position in map table
def cell_index_to_ij(cell_index):
    global width
    return (cell_index//width, cell_index%width)

# Given a current cell index and a goal cell index, convert the cost as the direct distance between them
def cost(cell_index_from, cell_index_to):
    global max_dist
    pos = cell_index_to_ij(cell_index_from)
    goal = cell_index_to_ij(cell_index_to)
    cost = math.sqrt((goal[0]-pos[0])**2 + (goal[1]-pos[1])**2)
    
    cost*=100/max_dist # set cost to within 0-100 range 
    return cost

if __name__ == "__main__":
    main()


