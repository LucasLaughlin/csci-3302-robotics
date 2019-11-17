#!/usr/bin/python
# CSCI 3302 Sparki Simulator Version 1.1
# Hastily authored by Brad Hayes (bradley.hayes@colorado.edu)
# If getting ImageTk errors, run: sudo apt-get install python-imaging-tk

import math
import rospy
import sys
import json
import argparse
import time
import copy
from PIL import Image, ImageTk
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, String, Float32MultiArray, Empty
import numpy as np

if sys.version_info[0] == 2:
  import Tkinter as tk # Python 2
else:
  import tkinter as tk # Python 3

g_namespace = ""


# Driver Constants
CYCLE_TIME = 0.05 # 50ms cycle time
MAP_RESOLUTION = 0.0015 # meters per pixel
MAP_SIZE_X = 1200 # Default map size in pixels
MAP_SIZE_Y = 800 # Default map size in pixels
SPARKI_SIZE_RADIUS = 0.08 # 0.08m radius == 6.29in diameter 
SPARKI_ULTRASONIC_MAX_DIST = .75 # 0.75m max range for ultrasonic sensor

# ***** SERVO POSITIONS ***** #
SPARKI_SERVO_LEFT = 80
SPARKI_SERVO_CENTER = 0
SPARKI_SERVO_RIGHT = -80
SPARKI_SPEED = 0.0278 # 100% speed in m/s
SPARKI_AXLE_DIAMETER = 0.085 # Distance between wheels, meters 
SPARKI_WHEEL_RADIUS = 0.03 # Radius of wheels, meters

MAP_PADDING = round(SPARKI_SIZE_RADIUS / MAP_RESOLUTION)


# ROBOT STATE VARS
g_pose = None # Pose2D
g_motors = None # [Float, Float] each Float in range [-1,1]
g_ir_sensors = None # 5 Int Array in 0-1000 intensity
g_us_sensor = None # Float (in centimeters)
g_servo_angle = None
g_world_starting_pose = None # Pose2D
g_ping_requested = None # Bool: include US distance in next state broadcast
g_world_obstacles = None # Grid to indicate presence of obstacles [0,255]. 0 is free, otherwise occupied
g_world_surface = None # Grayscale image of world surface [0,255] (e.g., line following map or equivalent). 0 is empty, otherwise has line

# Publishers and Subscribers
g_pub_state = None
g_pub_odom = None
g_sub_motors = None
g_sub_ping = None
g_sub_odom = None
g_sub_servo = None
g_sub_render = None

# Visualization
g_tk_window = None
g_tk_label = None
g_render_requested = None
g_render_image_base = None

def recv_motor_command(msg):
    global g_motors
    if len(msg.data) != 2:
        rospy.logerr("Sparki given motor commands with too many entries! (Got %d expected 2)" % len(msg.data))
        g_motors = [0,0] # Stop robot
        return
    g_motors = [msg.data[0], msg.data[1]]

def recv_ping(msg):
    global g_ping_requested
    g_ping_requested = True

def recv_render(msg):
    global g_render_requested
    g_render_requested = True

def set_odometry(msg):
    global g_pose
    g_pose = copy.copy(msg)
    g_pose.theta = -g_pose.theta # Negate theta

def set_servo(msg):
    global g_servo_angle
    new_angle = msg.data

    g_servo_angle = math.radians(max(SPARKI_SERVO_RIGHT, min(SPARKI_SERVO_LEFT,-new_angle))) # Negate theta

def get_ir_reading():
    # Find the IR readings 'underneath' the Sparki robot and return them
    # Find pixels halfway between the center and front of the robot
    global g_pose, g_world_surface, SPARKI_SIZE_RADIUS, MAP_RESOLUTION, MAP_SIZE_X, MAP_SIZE_Y
    center_point = [g_pose.x, g_pose.y]
    sparki_sensor_center_point_offset = 0.5 * SPARKI_SIZE_RADIUS
    forward_point = (center_point[0] + math.cos(g_pose.theta)*sparki_sensor_center_point_offset, \
                     center_point[1] + math.sin(g_pose.theta)*sparki_sensor_center_point_offset)

    # Find map locations that IR sensors are 'over' in (x,y) [meters]
    left_line_ir_point = (forward_point[0] - math.sin(g_pose.theta)*SPARKI_SIZE_RADIUS/2, forward_point[1] + math.cos(g_pose.theta)*SPARKI_SIZE_RADIUS/2)
    left_ir_point = (forward_point[0] - math.sin(g_pose.theta)*SPARKI_SIZE_RADIUS/4, forward_point[1] + math.cos(g_pose.theta)*SPARKI_SIZE_RADIUS/4)
    center_ir_point = (forward_point[0], forward_point[1])
    right_ir_point = (forward_point[0] + math.sin(g_pose.theta)*SPARKI_SIZE_RADIUS/4, forward_point[1] - math.cos(g_pose.theta)*SPARKI_SIZE_RADIUS/4)
    right_line_ir_point = (forward_point[0] + math.sin(g_pose.theta)*SPARKI_SIZE_RADIUS/2, forward_point[1] - math.cos(g_pose.theta)*SPARKI_SIZE_RADIUS/2)

    # Convert to pixel locations
    pixel_coords = [0] * 5
    for i, coord in enumerate([left_line_ir_point, left_ir_point, center_ir_point, right_ir_point, right_line_ir_point]):
      pixel_coords[i] = (int(coord[0] / MAP_RESOLUTION), int(coord[1] / MAP_RESOLUTION))
      if pixel_coords[i][0] < 0 or pixel_coords[i][0] >= MAP_SIZE_X or pixel_coords[i][1] < 0 or pixel_coords[i][1] >= MAP_SIZE_Y:
        rospy.logerr("IR Sensor is off the map! Coords: %s" % (str(pixel_coords[i])))
        return [1000] * 5

    # Look up pixel coords on world map
    ir_readings = [1000] * 5
    for i, coord in enumerate(pixel_coords):
      ir_readings[i] = int(1000 * (255. - g_world_surface[coord[1],coord[0]]) / 255.)

    return ir_readings

def get_ping_reading():
    # Return distance from robot of an object in the line-of-sight of ultrasonic sensor
    # Returns -1 if nonesparki-sim.zip
    global g_servo_angle, g_world_obstacles, g_pose, MAP_RESOLUTION, MAP_SIZE_X, MAP_SIZE_Y, SPARKI_ULTRASONIC_MAX_DIST

    # Cast a ray until hitting map boundary or SPARKI_ULTRASONIC_MAX_DIST meters away
    offset = [math.cos(g_pose.theta + g_servo_angle), math.sin(g_pose.theta + g_servo_angle)]

    # Sample 500 points along the sensor line
    num_increments = 500
    for incr in range(1,num_increments):
      distance = (float(incr) / num_increments) * SPARKI_ULTRASONIC_MAX_DIST # Distance from robot being checked
      test_point_meters = [g_pose.x + distance * offset[0], g_pose.y + distance * offset[1]]
      test_point_pixels = [int(test_point_meters[0] / MAP_RESOLUTION), int(test_point_meters[1] / MAP_RESOLUTION)]
      if test_point_pixels[0] < 0 or test_point_pixels[1] < 0 or test_point_pixels[0] >= MAP_SIZE_X or test_point_pixels[1] >= MAP_SIZE_Y:
        # print("Sampled off map: %s" % (str(test_point_pixels)))
        return -1 # Sampling off the map
      if g_world_obstacles[test_point_pixels[1],test_point_pixels[0]] > 0:
        return distance * SPARKI_ULTRASONIC_MAX_DIST # Obstacle found

    return -1


def load_img_to_bool_matrix(img_filename):
    global MAP_SIZE_X, MAP_SIZE_Y

    if img_filename is None:
        grid = np.zeros([800,1200])
        return grid

    img = Image.open(img_filename)

    MAP_SIZE_X = img.width
    MAP_SIZE_Y = img.height

    grid = np.zeros([img.height, img.width])
    for y in range(img.height):
        for x in range(img.width):
            pixel = img.getpixel((x,y))
            grid[y,x] = 255 - pixel[0] # Dark pixels have high values to indicate being occupied/having something interesting

    return grid

def init(args):
    # Initialize all publishers, subscribers, state variables
    global g_namespace, g_pose, g_ir_sensors, g_us_sensor, g_world_starting_pose, g_world_obstacles, g_world_surface, g_motors, g_ping_requested, g_servo_angle
    global g_pub_state, g_pub_odom, g_sub_motors, g_sub_ping, g_sub_odom, g_sub_servo, g_render_requested, g_sub_render
    global g_tk_window, g_tk_label, g_render_image_base

    g_namespace = args.namespace

    rospy.init_node("sparki_simulator_%s" % g_namespace)

    g_ir_sensors = [0]*5
    g_us_sensor = -1
    g_motors = [0.,0.]
    g_servo_angle = 0
    g_ping_requested = False
    g_world_starting_pose = Pose2D()
    g_world_starting_pose.x, g_world_starting_pose.y, g_world_starting_pose.theta = args.startingpose[0], args.startingpose[1], args.startingpose[2]
    g_pose = copy.copy(g_world_starting_pose)

    g_world_obstacles = load_img_to_bool_matrix(args.obstacles)
    g_world_surface = load_img_to_bool_matrix(args.worldmap)

    if (g_world_obstacles.size != g_world_surface.size):
      g_world_obstacles = load_img_to_bool_matrix(None)
      g_world_surface = load_img_to_bool_matrix(None)
      rospy.logerr("Obstacle map and surface map have different dimensions. Resetting to blank!")

    MAP_SIZE_X = g_world_obstacles.shape[1]
    MAP_SIZE_Y = g_world_obstacles.shape[0]

    g_pub_odom = rospy.Publisher("/%s/odometry" % g_namespace, Pose2D, queue_size=10)
    g_pub_state = rospy.Publisher('/%s/state' % g_namespace, String, queue_size=10)

    g_sub_motors = rospy.Subscriber('/%s/motor_command' % g_namespace, Float32MultiArray, recv_motor_command)
    g_sub_ping = rospy.Subscriber('/%s/ping_command' % g_namespace, Empty, recv_ping)
    g_sub_odom = rospy.Subscriber('/%s/set_odometry' % g_namespace, Pose2D, set_odometry)
    g_sub_servo = rospy.Subscriber('/%s/set_servo' % g_namespace, Int16, set_servo)
    g_sub_render = rospy.Subscriber('/%s/render_sim' % g_namespace, Empty, recv_render)  

    g_tk_window = tk.Tk()
    img = ImageTk.PhotoImage('RGB', (MAP_SIZE_X, MAP_SIZE_Y))
    g_tk_label = tk.Label(g_tk_window, image = img)
    g_tk_label.pack(fill="both", expand="yes")

    g_render_requested = True
    g_render_image_base = Image.new('RGB', (MAP_SIZE_X, MAP_SIZE_Y), color = 'white')
    for y_coord in range(0,MAP_SIZE_Y):
      for x_coord in range(0, MAP_SIZE_X):
        # Add line-following diagram as shades of black-to-red
        if g_world_surface[y_coord, x_coord] > 0:
          g_render_image_base.putpixel((x_coord, y_coord), (255-int(g_world_surface[y_coord, x_coord]/5),0,0))

        # Add objects as shades of black-to-green
        if g_world_obstacles[y_coord, x_coord] > 0:
          g_render_image_base.putpixel((x_coord, y_coord), (0,255-int(g_world_surface[y_coord, x_coord]/5),0))

def update_and_publish_state(pub):
  global g_pose, g_ir_sensors, g_us_sensor, g_world_starting_pose, g_world_obstacles, g_motors, g_ping_requested, g_servo_angle

  state = {}
  state['servo'] = math.degrees(-g_servo_angle) # Un-Negate theta
  state['light_sensors'] = get_ir_reading()

  if g_ping_requested is True:
    state['ping'] = get_ping_reading()
    g_ping_requested = False

  pub.publish(json.dumps(state))


def update_and_publish_odometry(pub, time_delta):
  global SPARKI_SPEED, SPARKI_AXLE_DIAMETER, MAP_SIZE_X, MAP_SIZE_Y
  global g_motors, g_pose
  left_wheel_dist = (g_motors[0] * time_delta * SPARKI_SPEED)
  right_wheel_dist = (g_motors[1] * time_delta * SPARKI_SPEED)

  g_pose.x += math.cos(g_pose.theta) * (left_wheel_dist+right_wheel_dist)/2. 
  g_pose.y += math.sin(g_pose.theta) * (left_wheel_dist+right_wheel_dist)/2. 
  g_pose.theta += (right_wheel_dist - left_wheel_dist) / SPARKI_AXLE_DIAMETER

  g_pose.x = min(MAP_SIZE_X*MAP_RESOLUTION, max(g_pose.x, 0))
  g_pose.y = min(MAP_SIZE_Y*MAP_RESOLUTION, max(g_pose.y, 0))

  pub_pose = copy.copy(g_pose)
  pub_pose.y = MAP_SIZE_Y * MAP_RESOLUTION - pub_pose.y
  pub_pose.theta = -pub_pose.theta # Negate theta
  pub.publish(pub_pose)

def render_robot_and_scene():
    global SPARKI_SIZE_RADIUS, g_tk_window, g_tk_label, g_render_image_base

    render_img = copy.copy(g_render_image_base)
    
    robot_pixel_coords = np.array( [int(g_pose.x / MAP_RESOLUTION), int(g_pose.y / MAP_RESOLUTION)] )

    # Add robot as blue
    robot_y_range = [int(robot_pixel_coords[1] - SPARKI_SIZE_RADIUS/MAP_RESOLUTION - 1), int(robot_pixel_coords[1] + SPARKI_SIZE_RADIUS/MAP_RESOLUTION + 1)]
    robot_x_range = [int(robot_pixel_coords[0] - SPARKI_SIZE_RADIUS/MAP_RESOLUTION - 1), int(robot_pixel_coords[0] + SPARKI_SIZE_RADIUS/MAP_RESOLUTION + 1)]
    for y_coord in range(robot_y_range[0], robot_y_range[1]):
      for x_coord in range(robot_x_range[0], robot_x_range[1]):
          if (np.linalg.norm(robot_pixel_coords - np.array((x_coord, y_coord))) < SPARKI_SIZE_RADIUS/MAP_RESOLUTION):
            render_img.putpixel((x_coord,y_coord), (0,0,128))

    # Add servo/ultrasonic direction as line extending halfway out sparki's radius
    sparki_pixel_radius = int(SPARKI_SIZE_RADIUS / MAP_RESOLUTION)
    for pct in range(0, 100):
      dist = pct/100. * sparki_pixel_radius / 2
      for width in range(-5,6,1):          
          pixel_coord = [ int(robot_pixel_coords[0] + width*math.cos(g_pose.theta) + math.cos(g_pose.theta + g_servo_angle) * dist), int(robot_pixel_coords[1] + width*math.sin(g_pose.theta) + math.sin(g_pose.theta + g_servo_angle) * dist) ]
          render_img.putpixel( pixel_coord, (255, 255, 0) )

    # Add small indicator for front of robot
    front_of_robot_pixel_coord = [ int(robot_pixel_coords[0] + math.cos(g_pose.theta)* sparki_pixel_radius), int(robot_pixel_coords[1] + math.sin(g_pose.theta)*sparki_pixel_radius) ]
    for y_coord in range(front_of_robot_pixel_coord[1] - 3, front_of_robot_pixel_coord[1] + 3 + 1):
      for x_coord in range(front_of_robot_pixel_coord[0] - 3, front_of_robot_pixel_coord[0] + 3 + 1):
        if x_coord < 0 or y_coord < 0 or x_coord >= MAP_SIZE_X or y_coord >= MAP_SIZE_Y:
          continue
        render_img.putpixel( (x_coord, y_coord), (0, 255, 255) )

    # Display on screen
    img = ImageTk.PhotoImage(render_img) #render_img.resize((320,240), Image.ANTIALIAS))
    g_tk_label.configure(image=img)
    g_tk_label.image = img
    g_tk_window.update_idletasks()
    g_tk_window.update()


def launch_simulator(args):
    global g_pub_odom, g_pub_state, g_render_requested
    init(args)

    last_time = time.time()
    while not rospy.is_shutdown():
        cycle_start = time.time()
        # Update and Publish Odometry
        update_and_publish_odometry(g_pub_odom, time.time() - last_time)
        last_time = time.time()
        # Update and Publish Sensors
        update_and_publish_state(g_pub_state)
        if g_render_requested is True:
          render_start_time = time.time()
          render_robot_and_scene()
          render_end_time = time.time()
          cycle_start += render_end_time - render_start_time
          last_time += render_end_time - render_start_time
          g_render_requested = False
        if CYCLE_TIME-(time.time()-cycle_start) < 0:
          rospy.loginfo("CYCLE TIME over by: %f" % (CYCLE_TIME-(time.time()-cycle_start)))
        rospy.sleep(max(0,CYCLE_TIME-(time.time()-cycle_start)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sparki Simulation Environment")
    # 800, 656 for bottom line on default line follow map
    default_x, default_y = 800 * MAP_RESOLUTION, 656 * MAP_RESOLUTION
    parser.add_argument('-n','--namespace', type=str, nargs='?', default='sparki', help='Prepended string for all topics')
    parser.add_argument('-p','--startingpose', nargs=3, default=[default_x, default_y, 0.], help='Starting x, y, theta of Sparki in world coords')
    parser.add_argument('-w','--worldmap', nargs='?', type=str, default='line-follow.png', help='Black and white image showing the ground texture of the world')
    parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles.png', help='Black and white image showing the obstacle locations')
    args = parser.parse_args()

    launch_simulator(args)