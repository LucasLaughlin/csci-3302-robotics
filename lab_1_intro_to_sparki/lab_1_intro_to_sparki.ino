#include <Sparki.h>; // include the sparki library
int state;
int cm;
int line_left;
int line_right;
int line_center;
int threshold;
void setup()
{
   sparki.servo(SERVO_CENTER);
   sparki.gripperOpen();
   delay(5000);
   sparki.gripperStop();
   delay(1000);
   sparki.clearLCD();
   threshold = 500;
   state=1;
}

void readSensors() {
  cm = sparki.ping();
  line_left = sparki.lineLeft(); 
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void loop()
{  
  readSensors();
  switch (state)
  {
      case 1: // Rotate until object is detected within 30 cm
          sparki.RGB(RGB_GREEN);
          sparki.moveRight(5);
          
          if(cm > 0){
            if (cm <= 30)
            {

              state = 2;
            }
          }
          break;
          
      case 2: // Move forward until object is 7 cm away;
          sparki.RGB(RGB_RED);
          sparki.moveForward();
          
          sparki.print(cm);
          if(cm>0){
            if (cm <= 7){
              sparki.print(cm);
              state = 3;
            }
          }
          break;
          
      case 3: // Grip object + spin 180 
          delay(2000);
          sparki.moveStop();
          sparki.RGB(RGB_BLUE);
          sparki.gripperClose();
          delay(5000); 
          sparki.gripperStop();
          delay(1000); 
          sparki.moveRight(180);         
          state=4;
          break;
          
       case 4: //move to line
          sparki.RGB(RGB_YELLOW);
          sparki.moveForward();
          
          if (line_center > threshold){
            sparki.moveStop();
            state=5;
          }
          break;
          
       case 5:
          sparki.RGB(RGB_GREEN);
          if ( (line_center < threshold) && (line_left < threshold) && (line_right < threshold) )
          {
            sparki.moveStop(); // move forward
            sparki.beep();
            sparki.gripperOpen();
            state = 6;
          }
          if ( line_left < threshold ) // if line is below left line sensor
          {  
            sparki.moveLeft(); // turn left
          }

          if ( line_right < threshold ) // if line is below right line sensor
          {  
            sparki.moveRight(); // turn right
          }

          // if the center line sensor is the only one reading a line
          if ( (line_center < threshold) && (line_left > threshold) && (line_right > threshold) )
          {
            sparki.moveForward(); // move forward
          }  
          break;
       case 6:
        	sparki.RGB(RGB_RED);
        	sparki.moveStop();
        	break;
      default: 
          break;// code to be executed if n doesn't match any case
  }
  //sparki.updateLCD();
  delay(100);
}
        