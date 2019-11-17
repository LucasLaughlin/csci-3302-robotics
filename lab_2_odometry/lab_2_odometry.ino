#include <Sparki.h>
#include <Math.h>

#define CYCLE_TIME .100  // seconds
#define AXLE_LENGTH 0.0857  // Meters -- Distance between wheels
#define SPEED .3/10.594

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_FOLLOW_LINE;//CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;
float d_x=0, d_y=0, d_theta=0;
unsigned long start;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  d_x=0;
  d_y=0;
  d_theta=0;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void measure_30cm_speed() { //10954 ms, 10954 ms, 10954 ms -> 10.594 s -> .30/10.594 m/s  
  sparki.clearLCD();
  sparki.moveForward(30);
  sparki.print(millis());
  sparki.updateLCD();
}


void updateOdometry(float d_x, float d_y, float d_theta) {
  pose_x+=d_x;
  pose_y+=d_y;
  pose_theta+=d_theta;
  return;
}

void displayOdometry() {
  sparki.clearLCD();
  sparki.print("x-pos: ");
  sparki.println(pose_x);
  sparki.print("y-pos: ");
  sparki.println(pose_y);
  sparki.print("theta: ");
  sparki.println((pose_theta / PI) * 180.0);
  sparki.updateLCD();
  
}

void followLine() {
  
  readSensors();
  
  if ((line_left < threshold) && (line_right < threshold) && (line_center < threshold))
      {
       sparki.RGB(RGB_BLUE);
       sparki.moveForward();
       pose_x=0;
       pose_y=0;
       pose_theta=0;
       return;
      }
  if ( line_left < threshold ) // if line is below left line sensor
      {  
       sparki.RGB(RGB_GREEN);
       sparki.moveLeft(); // turn left
       d_x =0;
       d_y =0;
       d_theta=2*(SPEED*.1)/AXLE_LENGTH;
      }
 
  if ( line_right < threshold ) 
      {  
       sparki.RGB(RGB_YELLOW);
       sparki.moveRight();
       d_x =0;
       d_y =0;
       d_theta=-2*(SPEED*.1)/AXLE_LENGTH;
      }
 

  if ( (line_center < threshold) && (line_left > threshold) && (line_right > threshold) )
      {
       sparki.RGB(RGB_RED);
       sparki.moveForward(); 
       d_x = cos(pose_theta)*(SPEED)*.1;
       d_y = sin(pose_theta)*(SPEED)*.1;
       d_theta=0;    
      }
  updateOdometry( d_x, d_y, d_theta);
  displayOdometry();
  
}

void loop() {
  start = millis();
  
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
    
      followLine();
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      current_state=3;
      break;
    default:
      break;
  }

  while(millis()< start+100){

  }
}
