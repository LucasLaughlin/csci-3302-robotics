#include <Sparki.h>; // include the sparki library
int state =1;
void setup()
{
   sparki.servo(SERVO_CENTER);
   sparki.gripperOpen();
   delay(5000);
   sparki.gripperStop();
   delay(1000);
}
 
void loop()
{
  int threshold = 500;
  sparki.clearLCD();
  switch (state)
  {
      case 1: // Rotate until object is detected within 30 cm
          sparki.RGB(RGB_GREEN);
          
          sparki.moveRight(10);
          int cm = sparki.ping();
          sparki.print(cm);
          if(cm > 0){
            if (cm <= 30)
            {
              sparki.print(cm);
              state = 2;
            }
          }
         break;
      case 2: // Move forward until object is 7 cm away;
          sparki.RGB(RGB_RED);
          sparki.moveForward(10);
          int cm2 = sparki.ping();
          sparki.print(cm);
          if(cm>0){
            if (cm2 <= 5){
              sparki.print(cm);
              state = 3;
            }
          }
          
          break;
      case 3: // Grip object + spin 180 
          sparki.RGB(RGB_BLUE);
          sparki.gripperClose();
          delay(5000); 
          sparki.gripperStop();
          delay(1000); 
          sparki.moveRight(180);
          
      default: 
          break;// code to be executed if n doesn't match any case
  }
  sparki.updateLCD();
  delay(100);
}
        
