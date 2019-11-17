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
const int current_state = CONTROLLER_GOTO_POSITION_PART3;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float x_err=0., y_err=0., d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)

float dlta_x=0.,dlta_y=0., dlta_theta=0.;

float dist_l=0., dist_r=0., dist_c=0., dist_out=0., dist_in=0., prcnt_l=0., prcnt_r=0.; 

float x_change=0., theta_change=0.;


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
  dlta_x=0;
  dlta_y=0;
  dlta_theta=0;

  //set_pose_destination(0.15,-0.05, to_radians(270));
  //set_pose_destination(-0.15,-0.05, to_radians(180));
  set_pose_destination(0.0,0.15, to_radians(90));
  //set_pose_destination(0.15,0.05, to_radians(135));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = 0; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}


void updateOdometry() {
  pose_x+=dlta_x;
  pose_y+=dlta_y;
  pose_theta+=dlta_theta;
  //d_x = cos(pose_theta)*(SPEED)*CYCLE_TIME;
  //d_y = sin(pose_theta)*(SPEED)*CYCLE_TIME;
  
  // Bound theta
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y); 
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(x_change );
  sparki.print("   dT: ");
  sparki.println(theta_change);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));  
}

void calculateError() {
  x_err = dest_pose_x-pose_x;
  y_err = dest_pose_y-pose_y;

  d_err = sqrt(pow(y_err,2) + pow(x_err,2));
  b_err = atan2(y_err,x_err);
  h_err = dest_pose_theta - pose_theta;
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
        sparki.moveForward(); 
        dlta_x = cos(pose_theta)*(SPEED)*.1;
        dlta_y = sin(pose_theta)*(SPEED)*.1;
        dlta_theta=0;
      } else if (line_left < threshold) {
        sparki.moveLeft(); // turn left
        dlta_x =0;
        dlta_y =0;
        dlta_theta=2*(SPEED*.1)/AXLE_DIAMETER;
      } else if (line_right < threshold) {
        sparki.moveRight();
        dlta_x =0;
        dlta_y =0;
        dlta_theta=-2*(SPEED*.1)/AXLE_DIAMETER;
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
        sparki.RGB(RGB_RED);

        calculateError();

        sparki.moveLeft(to_degrees(b_err));
        h_err-= b_err;
        
        sparki.RGB(RGB_BLUE);      

        sparki.moveForward(d_err*100);

        sparki.RGB(RGB_GREEN);

        sparki.moveLeft(to_degrees(h_err));
        sparki.RGB(RGB_YELLOW);
        
        pose_x=x_err;
        pose_y=y_err;
        pose_theta=h_err+b_err;
      }
      
      
      
      break;      
    case CONTROLLER_GOTO_POSITION_PART3:
      calculateError();
      
      
      if (d_err<0.05 && fabs(h_err) < to_radians(15)){ 
        sparki.moveStop();
        x_change=0;
        theta_change=0;
        sparki.RGB(RGB_RED);
        return;
      }
                     
      x_change = d_err;
      
      theta_change = (d_err>0.05) ? b_err: h_err;
       
      if (theta_change<0){theta_change+=2*M_PI;}
      if (theta_change>2*M_PI){theta_change-=2*M_PI;}

      
      phi_l = (x_change - (theta_change*AXLE_DIAMETER)/2)/WHEEL_RADIUS;
      phi_r = (x_change + (theta_change*AXLE_DIAMETER)/2)/WHEEL_RADIUS;
      
      if (fabs(phi_l)>fabs(phi_r)){
        prcnt_l = 100;
        prcnt_r = 100 * fabs(phi_r/phi_l);
      }
      else {
        prcnt_r = 100;
        prcnt_l = 100 * fabs(phi_l/phi_r);
      }

      left_dir = (phi_l>=0) ? DIR_CCW : DIR_CW;
      right_dir = (phi_r>=0) ? DIR_CW : DIR_CCW;
        
      sparki.motorRotate(MOTOR_LEFT, left_dir,  int(prcnt_l));
      sparki.motorRotate(MOTOR_RIGHT, right_dir,  int(prcnt_r));   

       //Odometry stuff:        
      dist_l = (prcnt_l * .01) * CYCLE_TIME * ROBOT_SPEED;
      dist_r = (prcnt_r * .01) * CYCLE_TIME * ROBOT_SPEED;
      
      dist_c = (dist_l + dist_r)/2;
      
      dlta_theta = (dist_l-dist_r)/AXLE_DIAMETER;
      
      dlta_x = dist_c * cos(pose_theta);
      dlta_y = dist_c * sin(pose_theta); //http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html

      updateOdometry();
      
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));

      break;
    default:
      break;
  }

  sparki.clearLCD();
  updateOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
 
