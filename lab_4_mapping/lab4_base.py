import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import math
from math import cos, sin
from Tkinter import * #http://effbot.org/tkinterbook/canvas.htm


# GLOBALS 
pose2d_sparki_odometry = Pose2D(0, 0, 0) #Pose2D message object, contains x,y,theta members in meters and radians
servo_angle = None
ping_distance = None
ir_sensor_readings = [0 for i in range(5)]
row = 10
col = 10
max_dist = math.sqrt(row**2 + col**2)
map_coord = [0 for i in range(row*col)]
map_cell_size = 5
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
        # Check for start line, use as loop closure
        #if (ir_sensor_readings[1] < IR_THRESHOLD and ir_sensor_readings[3] < IR_THRESHOLD and ir_sensor_readings[2]< IR_THRESHOLD):
            #zero= Pose2D()
            #zero.x = 0.0
            #zero.y = 0.0
            #zero.theta = 0.0
            #publisher_odometry.publish(zero)
        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))

        #TODO: Implement loop closure here
        if(render_count % 100000 == 0):
            publisher_render_sim.publish(Empty())
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

    rospy.init_node('sparki', anonymous=TRUE)

    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray) # Send two between [-1.0, 1.0] for left/right wheels
    publisher_ping = rospy.Publisher('/sparki/ping_command', Empty) # Tell sparki to ping ASAP
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16) # Tell Sparki to set servo motor to an angle between [-80, 80] degrees
    publisher_odometry = rospy.Publisher('/sparki/set_odometry', Pose2D) # Tell sparki to set odometry
    publisher_render_sim = rospy.Publisher('/sparki/render_sim',Empty)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state) # use json.loads to turn JSON dictionary to python object
    subscriber_odometry = rospy.Subscriber('odometry', Pose2D, callback_update_odometry) # contains x, y, theta

    rospy.sleep(0.2)
    
    # Set initial x, y, theta positions
    # pose2d_sparki_odometry.x = 0
    # pose2d_sparki_odometry.y = 0
    # pose2d_sparki_odometry.theta = 0
    # publisher_odometry.publish(pose2d_sparki_odometry) #publish initial position to odometry
    publisher_servo.publish(45)

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    pose2d_sparki_odometry.x = data.x
    pose2d_sparki_odometry.y = data.y
    pose2d_sparki_odometry.theta = data.theta
    robot_coord_to_map()
    pass
  
def callback_update_state(data):
    global servo_angle, ping_distance, ir_sensor_readings 
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic   
    # TODO: determine disctionary key values for state_dict
    servo_angle = state_dict['servo']
    ir_sensor_readings = state_dict['light_sensors']
    if ('ping' in state_dict):
        ping_distance = state_dict['ping']
        robot_coords = convert_ultrasonic_to_robot_coords(ping_distance)
        world_coords = convert_robot_coords_to_world(robot_coords[0], robot_coords[1])
        populate_map_from_ping(world_coords[0], world_coords[1])
    pass
  
# Given a ping distance, calculate x, y from robot frame of reference
def convert_ultrasonic_to_robot_coords(x_us):
    global pose2d_sparki_odometry
    x_r = x_us * cos(pose2d_sparki_odometry.theta)
    y_r = x_us * sin(pose2d_sparki_odometry.theta)
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
    map_coord[ij_to_cell_index(int(x_ping),int(y_ping))] = 1
    pass

def robot_coord_to_map():
    global pose2d_sparki_odometry, robot_map_index
    r_map_coord_x=pose2d_sparki_odometry.x//map_cell_size
    r_map_coord_y=pose2d_sparki_odometry.y//map_cell_size
    robot_map_index = ij_to_cell_index(r_map_coord_x, r_map_coord_y)

#R is the robot position
#O is an open spot
#X is a blocked spot
def display_map():
    global row, col, map_coord, robot_map_index
    #out = [["" for i in range(col)] for j in range(row)]
    out = []
    for i in range(10):
        out.append([])
        for j in range(col):
            out[i].append("")
    for i in range(10):
        for j in range(col):
            if (map_coord[ij_to_cell_index(i,j)]== 0):
                out[i][j]="O"
            elif (map_coord[ij_to_cell_index(i,j)]== 1):
                out[i][j]="X"
            if ([ij_to_cell_index(i,j)]==robot_map_index):
                out[i][j]="R"
    print('\n'.join([''.join(['{:4}'.format(item) for item in row]) for row in out]))
    print('\n\n')
    pass

# convert a x, y position in map table to a single value index of cell
def ij_to_cell_index(i,j):
    global col
    return (col*j + i)

# convert a cell index into an x, y position in map table
def cell_index_to_ij(cell_index):
    global col
    return (cell_index%col, cell_index//col)

# Given a current cell index and a goal cell index, convert the cost as the direct distance between them
def cost(cell_index_from, cell_index_to):
    global max_dist
    posx, posy = cell_index_to_ij(cell_index_from)
    goalx, goaly = cell_index_to_ij(cell_index_to)
    cost = math.sqrt((goalx-posx)**2 + (goaly-posy)**2)
    
    cost*=100/max_dist # set cost to within 0-100 range 
    return cost

if __name__ == "__main__":
    main()


