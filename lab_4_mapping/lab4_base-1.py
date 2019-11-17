/sparki/
    # find key values from rosimport rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


# GLOBALS 
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
servo_angle = None
ir_sensor_readings = [0 for i in 5]
rows, cols = (4, 5)
map_coord = [[0 for i in range(cols)] for j in range(rows)]

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_state = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.1 # In seconds

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    #TODO: Init your node to register it with the ROS core
    init()

    while not rospy.is_shutdown():
        #TODO: Implement CYCLE TIME

        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))

        #TODO: Implement loop closure here
        if False:
            rospy.loginfo("Loop Closure Triggered")

        #TODO: Implement CYCLE TIME
        rospy.sleep(0)



def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry

    rospy.init_node('sparki', anonymous=TRUE)

    publisher_motor = rospy.Publisher('sparkimotor_command', Float32MultiArray) # Send two between [-1.0, 1.0] for left/right wheels
    publisher_ping = rospy.Publisher('ping_command', Empty) # Tell sparki to ping ASAP
    publisher_servo = rospy.Publisher('set_servo', Int16) # Tell Sparki to set servo motot to an angle between [-80, 80] degrees
    publisher_odometry = rospy.Publisher('set_odometry', Pose2D) # Tell sparki to set odometry

    subscriber_state = rospy.Subscriber('state', String, callback_update_state) # use json.loads to turn JSON dictionary to python object
    subscriber_odometry = rospy.Subscriber('odometry', Pose2D, callback_update_odometry) # contains x, y, theta

    publisher_odometry.publish(pose2d_sparki_odometry)
    publisher_servo.publish(45)
    #TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    #TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 45)

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    pose2d_sparki_odometry.x = data.data.x
    pose2d_sparki_odometry.y = data.data.y
    pose2d_sparki_odometry.theta = data.data.theta

def callback_update_state(data):
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #TODO: Load data into your program's local state variables
    # CHECK dictiornary key values 
    servo_angle = state_dict[]
		ping_distance = state_dict[]
		ir_sensor_readings = state_dict[]
    
def convert_ultrasonic_to_robot_coords(x_us):
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = 0., 0.
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.

    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    pass

def display_map():
    #TODO: Display the map
    pass

def ij_to_cell_index(i,j):
    # Convert from i,j coordinates to a single integer that identifies a grid cell
    return j * rows + i

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    return (cell_index%rows, cell_index/rows)


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    return 0

if __name__ == "__main__":
    main()


