#include <Sparki.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1


// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
const int current_state = CONTROLLER_GOTO_POSITION_PART3; //CONTROLLER_GOTO_POSITION_PART2;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float x_err=0., y_err=0., d_err = 0., b_err = 0., h_err = 0.; // X component Distance error (x), y component Distance error (y), Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0.0000001, phi_r = 0.0000001; // Wheel rotation (radians)
float dist_l = 0., dist_r=0., dist_c=0.; //distance left wheel, right wheel, center move (meters)
float incr_theta = 0.0;

// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  //set_pose_destination(0.15,-0.05, to_radians(270));
  //set_pose_destination(-0.15,-0.05, to_radians(180));
  //set_pose_destination(0.0,0.15, to_radians(90));
  set_pose_destination(0.2,0.5, to_radians(-90));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta <= -M_PI) dest_pose_theta += 2*M_PI;
  calculateError(); //initialize d_err for following line
  orig_dist_to_goal = d_err; 
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}


void updateOdometry() {
  //distance left & right wheels moved
  dist_l = (left_speed_pct * .01) * CYCLE_TIME * ROBOT_SPEED * (phi_l/fabs(phi_l));
  dist_r = (right_speed_pct * .01) * CYCLE_TIME * ROBOT_SPEED* (phi_r/fabs(phi_r));
  //distance robot center moved
  dist_c = (dist_l + dist_r)/2;
  //value by which to increment theta
  //incr_theta = (dist_r-dist_l)/AXLE_DIAMETER;
  //increment position variables
  pose_x+= dist_c * cos(pose_theta + incr_theta/2);
  pose_y+= dist_c * sin(pose_theta + incr_theta/2);
  pose_theta+= (dist_r-dist_l)/AXLE_DIAMETER;;

  // Bound theta
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;


  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y); 
  sparki.print("T: ");
  sparki.print(round(to_degrees(pose_theta)));
  sparki.print(" Tg: ");
  sparki.println(round(to_degrees(dest_pose_theta)));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print(" dT: ");
  sparki.println(dTheta);
  sparki.print("phl:"); sparki.print(phi_l); sparki.print(" phr:"); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.print(to_degrees(b_err));
  sparki.println("h:"); sparki.println(to_degrees(h_err));  
  //sparki.print("dL: "); sparki.print(dist_l); sparki.print("dR: ");sparki.println(dist_r);
}

/*Calculate and update the global error variables based 
off current position relative to goal position*/
void calculateError() {
  //directional components of distance error
  x_err = dest_pose_x-pose_x;
  y_err = dest_pose_y-pose_y;

  //distance, bearing and heading error
  d_err = sqrt(pow(y_err,2) + pow(x_err,2));
  b_err =  atan2(y_err,x_err) - pose_theta;
  h_err = dest_pose_theta - pose_theta;

  //bound heading error to fit within theta bounds
  if (h_err > M_PI) h_err -= 2.*M_PI;
  if (h_err <= -M_PI) h_err += 2.*M_PI;

  if (b_err > M_PI) b_err -= 2.*M_PI;
  if (b_err <= -M_PI) b_err += 2.*M_PI;
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
   
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        sparki.moveLeft();
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      } 
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      if (pose_x==0 && pose_y==0){
        calculateError();//line 118
        //point sparki at goal/reduce b_err to 0
        sparki.moveLeft(to_degrees(b_err));
        //adjust h_err to account for rotation
        h_err-= b_err;
        //move to goal coordinates/reduce d_err to 0
        sparki.moveForward(d_err*100);
        //rotate to heading goal/reduce h_err to 0
        sparki.moveLeft(to_degrees(h_err));
        //set current position to goal positiion;
        pose_x=x_err;
        pose_y=y_err;
        pose_theta=h_err+b_err;
      }
      break;      
    case CONTROLLER_GOTO_POSITION_PART3:      
      //Stop conditions
      if (d_err<0.1 && fabs(h_err) < to_radians(15)){ 
        sparki.moveStop();
        sparki.RGB(RGB_RED);
        break;
      }
      
      updateOdometry();//line 87
      calculateError();//line 118
            
      //X update each loop
      dX = d_err;
      //if distance error > 0.5, minimize bearing error, else minimize heading error
      dTheta = (d_err>0.1) ? 2 * b_err: h_err;

      //dTheta = (dTheta<0) ? dTheta + (2 * M_PI): dTheta;
      
      //calculate individual wheel rotations based on the desired update parameters
      phi_l = (dX - (dTheta * AXLE_DIAMETER) / 2) / WHEEL_RADIUS;
      phi_r = (dX + (dTheta * AXLE_DIAMETER) / 2) / WHEEL_RADIUS;
      
      //if left wheel rotation is greater than right wheel, set it to 100% speed, otherwise adjust ratio
      left_speed_pct = (fabs(phi_l)>=fabs(phi_r))? 100: 100 * fabs(phi_l/phi_r);
      //if right wheel rotation is greater than left wheel, set it to 100 speed, otherwise adjust ratio
      right_speed_pct = (fabs(phi_r)>=fabs(phi_l))? 100: 100 * fabs(phi_r/phi_l);
      
      //adjust rotation direction to accommodate negative rotations(-phi_r || -phi_l)
      left_dir = (phi_l>=0) ? DIR_CCW : DIR_CW;
      right_dir = (phi_r>=0) ? DIR_CW : DIR_CCW;
      
      //move sparki
      sparki.motorRotate(MOTOR_LEFT, left_dir,  int(left_speed_pct));
      sparki.motorRotate(MOTOR_RIGHT, right_dir,  int(right_speed_pct));
      
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));

      break;
  }

  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
