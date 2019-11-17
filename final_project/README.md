# Final Project #
Abstract
	We are going to create a moving and interactive display through the use of a robot arm and projection mapping. The robot arm will hold a blank white sheet that will move around and the projector will map to that screen as it moves. We would like the screen to recognize when someone is looking at it and then face that person. 



### Equipment ###
* USB Camera
* Depth Camera
* Rethink Robotics Sawyer Arm

### Deliverables and Implementation Plan ###
1. Create Face tracking Vision System - Lead: 	Jordan Smart Deadline: 11/23/19	
    * Install OpenCV Python package and verify it works with camera.
    * Install dlib library and verify it works by capturing facial landmarks.
    * Devise way to to tell when a face is facing directly toward a camera.
        * Equal distance between edge of face and eyes, mouth, nose?
    * Test: Can output a signal when a human face looks at it within 30 degrees of error
2. Robot arm control - Lead: Lucas Laughlin Deadline:  11/23/19
    * Get robot to obey movement commands form ROS
    * Figure out inverse kinematics to point the robot arm at a specified point
    * Attach Projection surface to Robot Arm - Lead: Jordan Smart  Deadline:	11/23/19
3. Create a lightweight projection surface
    * Devise method of attachment to robot arm
    * Move robot arm and ensure it stays tight
4. Create projection mapping function to map output onto a moving surface - Lead: Casey Tran Deadline: 11/23/19
    * Figure out a way to shift, rotate, distort a projected image in real time
