
#include <gfxfont.h>
#include <Adafruit_GFX.h>

#include <Wire.h>


  #include <NewPing.h>
  #include <Adafruit_LEDBackpack.h>

byte FWD = B00100010;  //ok
byte REV = B10001000; 
byte rotRight = B00000000; //ok
byte rotLeft = B10101010; //ok
byte strRight = B00101000; //ok
byte strLeft = B10000010; //ok

byte All = B10101010;
// advanced motion variables
byte motionRear = B00010001;//Rear
byte motionFront = B01000100;//front
byte motion45Left = B00000101; // motion rightwheels
byte motion45Right = B01010000;
byte motionRightWheels = B00010001;// RIGHT WHEELS
byte motionLeftWheels = B01000100;// LEFT WHEELS

//%%%%%%%%%%%% CONVERSION FACTORS %%%%%%%%%%%%%%%%%%%%%%
int Steps;
float steps_per_inch = 217.6;
float steps_per_degree = 24;
byte motion = B01010101;
float dist_wall;
int period = 1000;


//********** sensor variables

// starboard and port

  #define TRIGGER_SensorLF 34     // ?
  #define ECHO_SensorLF 35        // ?
  #define TRIGGER_SensorLB 30     // ?
  #define ECHO_SensorLB 31        // ?
  #define TRIGGER_SensorRB 22     // ?
  #define ECHO_SensorRB 23        // ?
  #define TRIGGER_SensorRF 26     // ok
  #define ECHO_SensorRF 27        // ok

// fore and aft  

  #define TRIGGER_SensorFR 28   //ok  
  #define ECHO_SensorFR 29      //ok 
  #define TRIGGER_SensorFL 36     //?
  #define ECHO_SensorFL 37        //?
  #define TRIGGER_SensorBL 32     //?     
  #define ECHO_SensorBL 33        //?
  #define TRIGGER_SensorBR 24     //?
  #define ECHO_SensorBR 25       //? 
  
  #define Max_Distance   200

  NewPing sonarFL(TRIGGER_SensorFL, ECHO_SensorFL,Max_Distance);
  NewPing sonarBL(TRIGGER_SensorBL, ECHO_SensorBL,Max_Distance);
  NewPing sonarBR(TRIGGER_SensorBR, ECHO_SensorBR,Max_Distance);
  NewPing sonarFR(TRIGGER_SensorFR, ECHO_SensorFR,Max_Distance);

  NewPing sonarLF(TRIGGER_SensorLF, ECHO_SensorLF,Max_Distance);
  NewPing sonarLB(TRIGGER_SensorLB, ECHO_SensorLB,Max_Distance);
  NewPing sonarRF(TRIGGER_SensorRB, ECHO_SensorRB,Max_Distance);
  NewPing sonarRB(TRIGGER_SensorRF, ECHO_SensorRF,Max_Distance);

  float micro_to_inches = .006756;
 
//************************************************* LED MATRIX  
int x;
int y;
int board[7][7]= {{0,0,0,0,0,0,0},
                  {0,0,0,0,0,0,0},
                  {0,0,0,0,0,0,0},
                  {0,0,0,0,0,0,0},
                  {0,0,0,0,0,0,0},
                  {0,0,0,0,0,0,0},
                  {0,0,0,0,0,0,0}};
Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();
//**************************************************SETUP
void setup() 
{
  Serial.begin(9600);
  // put your setup code here, to run once:
byte allOutputs = B11111111;
matrix.begin(0x70);
DDRL = allOutputs;

//************** test

//*************** search pattern

x = 1;
y = 1;



//*****test section

  
//********************basic motion functions
//ForwardBackward (12,0,period);  //ok, distance calibrated. 12 inches= 110 steps (distance in inches, 1 fwd, 0 bwd, delay)
//strafe(12,1,50);            // (distance, right/left, delay
 //turn(180,1,100);
//advanced motion
//angleMotion (20,1,1,300);      // (distance, lerft/right , fw/bw/ , delay)
//arcMotion(90,1,1,300);       // (radius, arc degrees, left/right, fw/bw, delay)
//pivotMotion(90,1,1,300);    // (radius, arc degrees, left/right, fw/bw, delay)

//variable speed functions
//varSpeed (600,r,12,0); //calibrated 95%ish   (delay, speed ratio, distance, direction) (2.08 ratio/ 105 inches for almost perfect 12 in radius circle)

//final variable speed plus acceleration. working ok

//Steps = 1;
//float del;
//for (int i=1; i<Steps; i++)
//  { 
//    del=800/i;
//    varSpeed (del,r,i*.2,1);
//  }
//float dis = 72-Steps*(Steps+1)*.1;
//    varSpeed (del,r,dis,1);

//for (int i=Steps; i>0; i--)
//  { 
//    del=800/i;
//    varSpeed (del,r,i*.2,1);
//  }




//sonar ();

//alignLeft();
//alignBack();

//float wall = dist_wall;

//for (int i=0;i<6;i++)
//{
//varSpeed (800,.993,12,1);
//alignLeft();
//float angle1 = (dist_wall-wall)*micro_to_inches;
//angle1 = atan(angle1/12)*180/3.1415926;
//if (angle1 > 0) turn(angle1,0,800);
//  else turn(-angle1,1,800); 
  
//}
}

void loop() 
{


}

void gridSearch()
{
matrix.clear();  
ForwardBackward(12,1,period);
turn(90,1,period);
ForwardBackward(12,1,period);
turn(90,0,period);
checkEmAll();
markHere(x,y);
while ((x < 5) || (y < 5))
  {
    if ( (y % 2) != 0 && x < 5)
        {
          ForwardBackward(12,1,period);
          x++;
          //check obstacle
          //check tick tracer / dead end
          //display result
          markHere(x,y);
        }
    else if ( (y % 2) == 0 && x > 1)
    {
      ForwardBackward(12,1,period);
      x--;
                //check obstacle
          //check tick tracer / dead end
          //display result
          markHere(x,y);
    }
    else if (x == 5 && y%2 != 0)
      {
        turn(90,1,period);
        ForwardBackward(12,1,period);
        turn(90,1,period);
        checkEmAll();
        y++;
          //check obstacle
          //check tick tracer / dead end
          //display result
          markHere(x,y);
      }
    else if (x == 1 && y%2==0)
      {
      turn(90,0,period);
      ForwardBackward(12,1,period);
      turn(90,0,period);
      checkEmAll();
      y++;
      //check obstacle
      //check tick tracer / dead end
      //display result
      markHere(x,y);
      }  
   }
}

void checkEmAll()
{
  AngleLeft();
  AngleRight();
  AngleBack();
  AngleFront();
}
//****************** align to wall

 
void alignLeft ()
 { float angle =0;
  angle = angleMeasurement(16);
  if (angle > 0) turn(angle,0,period);
  else turn(-angle,1,period); 
 }


//*************************************************************Function for a bunch of variables for sensors


float angleMeasurement(int measures) 
{
 long distFL=0;
 float angle;
  long distBL=0;
  int count=0;
  float threshold;
  float sumAngle=0;
  float dif = 0;
  // take 9 reading and use the average
  
  distFL += sonarFL.ping_median(9);
  distBL += sonarBL.ping_median(9);

  threshold = distFL-distBL;
  dist_wall +=(distFL+distBL)/2;

 dist_wall *=micro_to_inches;

 // getting rid of readings that are anomalies by comparing them to threshold, which is the difference between the two sensors on one side
 for (int i=0; i<measures; i++)
  {
    delay(30);
    distFL = sonarFL.ping();
    delay(30);
    distBL = sonarBL.ping();
  
    dif = (distFL-distBL);
    if ((dif>(threshold-30) )&& (dif<(threshold+30)))
      {
        sumAngle +=dif;
        count++;
      }
  }
  if (count != 0)    sumAngle /= count;    

 dif = sumAngle;
  dif *= micro_to_inches; //micro_to_inches is ised to determine the angle. we need a value in inches so we can perform the arcsin calculation

// calculating the angle between a wall and the side of the robot based on the difference between the readings of the two sensors on that side    
  if (dif>0)
       angle = asin(dif/3.75)*180/3.14159265;
      
    if (dif<0)
      angle = asin(dif/3.75)*180/3.14159265;
  

 return(angle); 
}



//**********************************************************Calculate angle and allign (left side)

void AngleLeft()
{
  long distLF=0;
  long distLB=0;
  for (int i =0 ; i < 5; i++)
  {
    distLF += sonarLF.ping_median(5);
    distLB += sonarLB.ping_median(5);
  }
  float dif = (distLF-distLB)/5*micro_to_inches;
  float angle;
    if (dif>0)
       angle = asin(dif/3.75)*180/3.14159265;
      
    if (dif<0)
      angle = asin(dif/3.75)*180/3.14159265;
        
  Serial.print("angle left: ");
  Serial.print(angle);
  Serial.println(" degrees");

  if (angle>3)
    turn(angle,0,period);
  if (angle<-3)  
     turn(abs(angle),1,period);
    
  
}

//**********************************************************Calculate angle and allign (right side)

void AngleRight()
{
  long distRF=0;
  long distRB=0;
  for (int i =0 ; i < 5; i++)
  {
    distRF += sonarRF.ping_median(5);
    distRB += sonarRB.ping_median(5);
  }
  float dif = (distRF-distRB)/5*micro_to_inches;
  float angle;
    if (dif>0)
       angle = asin(dif/3.75)*180/3.14159265;
      
    if (dif<0)
      angle = asin(dif/3.75)*180/3.14159265;
        
  Serial.print("angle right: ");
  Serial.print(angle);
  Serial.println(" degrees");

  if (angle>3)
    turn(angle,0,period);
  if (angle<-3)  
     turn(abs(angle),1,period);
    
  
}

//**********************************************************Calculate angle and allign (front side)

void AngleFront()
{
  long distFL=0;
  long distFR=0;
  for (int i =0 ; i < 5; i++)
  {
    distFL += sonarFL.ping_median(5);
    distFR += sonarFR.ping_median(5);
  }
  float dif = (distFL-distFR)/5*micro_to_inches;
  float angle;
    if (dif>0)
       angle = asin(dif/3.75)*180/3.14159265;
      
    if (dif<0)
      angle = asin(dif/3.75)*180/3.14159265;
        
  Serial.print("angle front: ");
  Serial.print(angle);
  Serial.println(" degrees");

  if (angle>3)
    turn(angle,0,period);
  if (angle<-3)  
     turn(abs(angle),1,period);
    
  
}
//**********************************************************Calculate angle and allign (Back side)

void AngleBack()
{
  long distBR=0;
  long distBL=0;
  for (int i =0 ; i < 5; i++)
  {
    distBR += sonarBR.ping_median(5);
    distBL += sonarBL.ping_median(5);
  }
  float dif = (distBR-distBL)/5*micro_to_inches;
  float angle;
    if (dif>0)
       angle = asin(dif/3.75)*180/3.14159265;
      
    if (dif<0)
      angle = asin(dif/3.75)*180/3.14159265;
        
  Serial.print("angle back: ");
  Serial.print(angle);
  Serial.println(" degrees");

  if (angle>3)
    turn(angle,0,period);
  if (angle<-3)  
     turn(abs(angle),1,period);
    
  
}




//*************************************************************45ANGLE Motion

void angleMotion (float inches, int dirLR, int dirFB, int Delay)
{
  long distance;
  inches *=steps_per_inch;
  distance = inches;
  
  if (dirFB==1) //dirFB=1 means forward, 0 means backward
    {
      PORTL = FWD;
      if (dirLR==1) //dirLR=1 means right, 0 means left
         for (int i=0; i<distance; i++)
          {
             PORTL ^= motion45Right;
             delayMicroseconds(Delay);
          }
      else
        for (int i=0; i<distance; i++)
           { 
            PORTL ^= motion45Left;
            delayMicroseconds(Delay);
           }        
    }
  else
    {
      PORTL = REV;
      if (dirLR==1) //dirLR=1 means right, 0 means left
         for (int i=0; i<distance; i++)
          {
             PORTL ^= motion45Right;
             delayMicroseconds(Delay);
          }
      else
        for (int i=0; i<distance; i++)
           { 
           PORTL ^= motion45Left;
            delayMicroseconds(Delay);
           }    
    }
}


//****************************************************************TURN

void turn (int deg, int dir, int Delay)
{
  long steps;
  
 steps = deg*steps_per_degree;
  if (dir==1) // dir=1 is right, dir=0 is left
    {
        PORTL = rotRight;
        for (int i=0; i<steps; i++)
          {
            PORTL ^= motion;
            delayMicroseconds(Delay);
          }
    }
   else
    {
      PORTL = rotLeft;
      for (int i=0; i<steps; i++)
        {
            PORTL ^= motion;
            delayMicroseconds(Delay);
        }
    }
}


//***********************************************************FORWARDBACKWARD

void ForwardBackward (float inches, int dir, int Delay)
{
   long distance;   
   inches *= steps_per_inch;
   distance = inches;
  
   
   if (dir==1) // dir=1 is forward, dir=0 is backward
    {
      PORTL = FWD;
    for (int i=0; i<distance; i++)
      { 
        PORTL ^= motion;
        delayMicroseconds (Delay);
      }
    }
  else
    {
      PORTL = REV;
    
    for (int i=0; i<distance; i++)
      {
        PORTL ^= motion;
        delayMicroseconds (Delay);
      }
    }
}
//************************************************************STRAFE

void strafe(float inches, int dir, int Delay)
{
  long distance;
  inches *=steps_per_inch;
  distance = inches;
  
  if (dir==1)  //dir=1 strafe right, if dir = 0 strafe left
    {
      PORTL = strRight;
      for (int i=0; i<distance; i++)
        {
          PORTL ^= motion;
          delayMicroseconds(Delay);
        }
    }
   else
    {
      PORTL = strLeft;
      for (int i=0; i<distance; i++)
        {
          PORTL ^= motion;
          delayMicroseconds(Delay);
        }
    }
}

//******************************************************** LED DISPLAYS

void displayLED(){
  
  for(int i = 0; i <=7; i++){
    for(int j = 0; j <=7; j++){
      matrix.drawPixel(i,j, LED_GREEN);
      matrix.writeDisplay();  
    }//end of second for
  }//end of first for
}
void markHere(int i, int j){
  //i += 1;
  //j += 1;
  matrix.drawPixel(i,j, LED_RED);
  matrix.writeDisplay();
}//end of marking the wire

