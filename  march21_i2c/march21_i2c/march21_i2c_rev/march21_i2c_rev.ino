#include <gfxfont.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_LEDBackpack.h>
#include <stdint.h>

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
  #define TRIGGER_SensorRB 26     // ?
  #define ECHO_SensorRB 27        // ?
  #define TRIGGER_SensorRF 22     // ok
  #define ECHO_SensorRF 23        // ok

// fore and aft  

  #define TRIGGER_SensorFR 28   //ok  
  #define ECHO_SensorFR 29      //ok 
  #define TRIGGER_SensorFL 36     //?
  #define ECHO_SensorFL 37        //?
  #define TRIGGER_SensorBL 32     //?     
  #define ECHO_SensorBL 33        //?
  #define TRIGGER_SensorBR 24     //?
  #define ECHO_SensorBR 25       //? 
  
  #define Max_Distance   100 
  #define arrayIndex 1200        //may have to either decrease or increase the number of sample data reading that we take
 
  int thresHold = 513;          //if analogRead is greater than this value then increase counter
  int thresHoldTick = 300;
  
  int getData[arrayIndex] = {0};
  static int fiveData[6000] = {0};                              //this is the sample data
  
  int counter = 0;              //counts of signal going above the threshold
  int micPin= 0;                 //microphone pin number
  int tickPin = 1;
  
  int thumperPin = 50;          //thumper pin number
  
  long averaging = 0;
  
  long timerA;
  long timerB;
  
  NewPing sonarFL(TRIGGER_SensorFL, ECHO_SensorFL,Max_Distance);
  NewPing sonarBL(TRIGGER_SensorBL, ECHO_SensorBL,Max_Distance);
  NewPing sonarBR(TRIGGER_SensorBR, ECHO_SensorBR,Max_Distance);
  NewPing sonarFR(TRIGGER_SensorFR, ECHO_SensorFR,Max_Distance);

  NewPing sonarLF(TRIGGER_SensorLF, ECHO_SensorLF,Max_Distance);
  NewPing sonarLB(TRIGGER_SensorLB, ECHO_SensorLB,Max_Distance);
  NewPing sonarRF(TRIGGER_SensorRB, ECHO_SensorRB,Max_Distance);
  NewPing sonarRB(TRIGGER_SensorRF, ECHO_SensorRF,Max_Distance);

  float micro_to_inches = 0.006756;
  float pi = 3.14159265;
  float sideWidth = 7.0675/micro_to_inches;
  float frontWidth = 6.75/micro_to_inches;
  
  float distanceXY;
  
  float oneFoot = 12/micro_to_inches;
  float Ycentered = 0.75/micro_to_inches;
  float Xcentered = 1.25/micro_to_inches;
  
  int Yoffset;
  int Xoffset;

  union byteToInt {byte input[4]; uint32_t out;};

  byteToInt byte_to_int;

  boolean gotData = 0;
  
  int byteCount = 0;
  
  const int FFT_SIZE = 256;

  uint32_t magnitudes[FFT_SIZE];
  
  int fft_bin = 0;
  
  boolean newMeas = false;
  
  int checkCount = 6;
  
  int sanityBoard[5][5]{{0,0,0,0,0},
                        {0,0,0,0,0},
                        {0,0,0,0,0},
                        {0,0,0,0,0},
                        {0,0,0,0,0}};
                        
    // Define various ADC prescaler
    const unsigned char PS_16 = (1 << ADPS2);
    const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
    const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
    const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
   
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

//8************************************************* COMMAND PINS
int commandPin = 52;
//**************************************************SETUP
void setup() 
{
  pinMode(thumperPin, OUTPUT);
  
//  Wire.begin(8);
//  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
  //analogReadResolution(10);
  byte allOutputs = B11111111;
  matrix.begin(0x70);
  DDRL = allOutputs;
  DDRB = allOutputs;
  DDRK = allOutputs;
  
  analogReference(INTERNAL2V56);
  
  int fuck = 0;
  int your = 10;
  int self = 0;
  for(int go = fuck; go < your; go++){
    self = analogRead(0);
    int douche = analogRead(1);
  }
  
  PORTK = 0x00;
  PORTB = 0x00;
  
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_64;

x = 0;
y = 0;

delay(200);



//gridSearch();


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
//  else turn(-angle1,1,800); 
  
//}

delay(2000);
powerTick(500);
delay(2000);
powerTick(2500);
}

void loop() 
{
//  if(newMeas){
//    Serial.print("\n");
//    Serial.print("Source:,");             //Which analog input source was that?
//    Serial.print("mic"); 
//    Serial.print(","); 
//    for(int i=0;i<FFT_SIZE;i++)
//    {
//      Serial.print(magnitudes[i]);
//      Serial.print(",");
//    }  
//      fft_bin = 0;
//      newMeas = false;
//}
//Serial.print("thump");

//for(int k = 0; k<(arrayIndex*5); k++)
//{
//  if(fiveData[k]>Threshold)
//  {
//    counter++;
//  }
//}
//Serial.println(counter);
////counter = 0;
//delay(1000);

//Serial.print("counter = ");


//counter =  0;



tickTracer();



   ///////////EXAMPLE OUTPUT////////////////

//Serial.print("Left Front = ");
//Serial.println(sonarLF.ping_median(5));
//
//Serial.print(" Left Back = ");
//Serial.println(sonarLB.ping_median(5));

//Serial.print(" Right Front = ");
//Serial.println(sonarRF.ping_median(5));
//
//Serial.print(" Right Back = ");
//Serial.println(sonarRB.ping_median(5));
//
//Serial.print(" Front Left = ");
//Serial.println(sonarFL.ping_median(5));
//
//Serial.print(" Front Right = ");
//Serial.println(sonarFR.ping_median(5));
//
//Serial.print(" Back Left = ");
//Serial.println(sonarBL.ping_median(5));
//
//Serial.print(" Back Right = ");
//Serial.println(sonarBR.ping_median(5));

//checkEmAll();
}

int thump(int number){

    int decision;
    PORTB ^= 0x08;
    delay(20);
    int counter = 0;
    timerA = micros(); 
    for (int i =0; i<arrayIndex;i++){ 
      int value = analogRead(micPin);
      if(value > thresHold){
          counter ++;
        }
    }
      if(counter > 80){
          Serial.println("Hollow");
          decision = 1;
        }else if(counter < 70 && counter > 50){
            Serial.println("Solid");
            decision = 2;
          }else{
             Serial.println("Not Sure");
             decision = 3;
            }

     delay(50);
    PORTB ^= 0x08; //Down
    //delay(24)
  
    
     timerB = micros();
     delay(200);
     
     Serial.print("Counter: ");
     Serial.println(counter);
     
     return decision;
    
}

int tickTracer(){
    
    int decision;
    int counter = 0;
    timerA = micros(); 
    for (int i =0; i<arrayIndex;i++){ 
      int value = analogRead(tickPin);
      if(value > thresHoldTick){
          counter ++;
        }
      //Serial.println(value);
    }
    if (counter > 80)
    {
      //Serial.println("wire");
      decision = 1;
    }
    else
    {
      //Serial.println("no wire");
      decision = 2;
    }
      
    
     timerB = micros();
     delay(200);
     
     Serial.print("Counter: ");
     Serial.println(counter);
     

    
    return decision;
}

void powerTick(int TIMECOP){
      digitalWrite(51,HIGH);
    delay(TIMECOP);
    digitalWrite(51,LOW);
}
    
void gridSearch()
{
matrix.clear();  
ForwardBackward(12,1,period);
y++;
turn(90,1,period);
ForwardBackward(12,1,period);
x++;
turn(90,0,period);
checkEmAll();
markHere(x,y);
while ((x < 5) || (y < 5))
  {
    if ( (x % 2) != 0 && y < 5)
        {
          ForwardBackward(12,1,period);
          y++;
          positionUpdate();
          //check obstacle
          //check tick tracer / dead end
          //display result
          markHere(x,y);
        }
    else if ( (x % 2) == 0 && y > 1)
    {
      ForwardBackward(12,0,period);
      y--;
      positionUpdate();
                //check obstacle
          //check tick tracer / dead end
          //display result
          markHere(x,y);
    }
    else if (y == 5 && x%2 != 0)
      {
        turn(90,1,period);
        ForwardBackward(12,1,period);
        turn(90,0,period);
        checkEmAll();
        x++;
        positionUpdate();
          //check obstacle
          //check tick tracer / dead end
          //display result
          markHere(x,y);
      }
    else if (y == 1 && x%2==0)
      {
      turn(90,1,period);
      ForwardBackward(12,1,period);
      turn(90,0,period);
      checkEmAll();
      x++;
      positionUpdate();
      //check obstacle
      //check tick tracer
       // thump
      //display result
      markHere(x,y);
      }  
   }
}

void checkEmAll()
{
  
  int yTolerance = 40; //microseconds
  
  float angleFront;
  float frontDistance;
  float angleLeft;
  float leftDistance;
  float angleBack;
  float backDistance;
  float angleRight;
  float rightDistance;

  float minMag;
   
//  Serial.print("front ");
//  Serial.println(angleFront);
//  Serial.print("left ");
//  Serial.println(angleLeft);
//  Serial.print("back ");
//  Serial.println(angleBack);
//  Serial.print("right ");
//  Serial.println(angleRight);
//  Serial.println("-------------------");
//  Serial.print("FRONT ");
//  Serial.println(frontDistance);
//  Serial.print("LEFT ");
//  Serial.println(leftDistance);
//  Serial.print("BACK ");
//  Serial.println(backDistance);
//  Serial.print("RIGHT ");
//  Serial.println(rightDistance);
//  Serial.println("-------------------");
//  if (angleFront == 100)
//    angleFront = angleBack;
//  if (angleBack == 100)
//    angleBack = angleFront;
//  if (angleRight == 100)
//    angleRight = angleLeft;
//  if (angleLeft == 100)
//    angleLeft = angleRight;
  
//  minMag += angleFront;
//  minMag += angleRight;
//  minMag += angleLeft;
//  minMag += angleBack;
//  
//  minMag /= 4;
  
//  if(angleFront != angleFront)
//  {
//    minMag = 45; 
//  }
//  else
//  {
//    minMag = abs(angleFront);
//  }
//  
//  }
//  else if ((abs(minMag) > abs(angleBack)) && !(angleBack != angleBack))
//  {
//    minMag = abs(angleBack);
//  }

  if ( y < 3)
  {
    angleBack = angleMeas(sonarBR, sonarBL, frontWidth);
    backDistance = distanceXY;
    Yoffset = (backDistance-Ycentered)-oneFoot*y;
    if ( x < 3)
    {
      angleLeft = angleMeas(sonarLB, sonarLF, sideWidth);
      leftDistance = distanceXY;
      Xoffset = (leftDistance-Xcentered)-oneFoot*x;
      minMag = (angleLeft+angleBack)/2; //quadrant 3
    }
    else       
    {
      angleRight = angleMeas(sonarRF, sonarRB, sideWidth);
      rightDistance = distanceXY;
      Xoffset = ((rightDistance-Xcentered)/(6-x))-oneFoot;
      minMag = (angleRight+angleBack)/2; //quadrant 4
    }
  }
  else 
  {
    angleFront = angleMeas(sonarFL, sonarFR, frontWidth);
    frontDistance = distanceXY;
    Yoffset = ((frontDistance-Ycentered)/(6-y))-oneFoot;
    if (x < 3)
    {
      angleLeft = angleMeas(sonarLB, sonarLF, sideWidth);
      leftDistance = distanceXY;
      Xoffset = (leftDistance-Xcentered)-oneFoot*x;
      minMag = (angleLeft+angleFront)/2; //quadrant 2
    }
    else
    {
      angleRight = angleMeas(sonarRF, sonarRB, sideWidth);
      rightDistance = distanceXY;
      Xoffset = ((rightDistance-Xcentered)/(6-x))-oneFoot;
      minMag = (angleRight+angleFront)/2; //quadrant 1
    }
  }
  
  if (minMag>1)
  {
    turn(minMag, 1, period);
  }
  else if (minMag < -1)
  {
    turn(abs(minMag), 0, period);
  }

  if (Yoffset > 0)
  {
    ForwardBackward(Yoffset*micro_to_inches,0,period);
  }
  else
  {
    ForwardBackward(abs(Yoffset*micro_to_inches),1,period);
  }
  if (Xoffset > 0)
  {
    //turn(90,1,period);
    //ForwardBackward(Yoffset*micro_to_inches,0,period);
    //turn(90,0,period);
    strafe(Xoffset*micro_to_inches,0,period);
  }
  else
  { 
    //turn(90,1,period);
    //ForwardBackward(Yoffset*micro_to_inches,1,period);
    //turn(90,0,period);
    strafe(abs(Xoffset*micro_to_inches),1,period);
  } 
  
 

 if((checkCount % 2) == 0)
 {
   matrix.drawPixel(7,7, LED_RED);
   matrix.writeDisplay();
   checkCount++;
 }
 else
 {
   matrix.drawPixel(7,7, LED_GREEN);
   matrix.writeDisplay();
   checkCount--;
   
 }

}

////////////////////////////////////////////////////////// RECIEVE EVENT HANDLER
void receiveEvent(int howMany){
  if(howMany){
    for(int i=0; i<howMany; i++){
        byte_to_int.input[i] = Wire.read();
      }
      magnitudes[fft_bin] = byte_to_int.out;
        fft_bin++;    
   if(fft_bin == FFT_SIZE)
   { 
     fft_bin = 0;
     newMeas = true;
   }    
  }
}
//****************** align to wall

//**********************************************************Calculate angle
// firts sensor is MOST COUNTERCLOCKWISE
float angleMeas(NewPing sens1, NewPing sens2, float sensWidth)//sensWidth front  = 6.75, sensWidth sides = 7.0625
{
  int N = 1;
  float dist1 = 0;
  float dist2 = 0;
  float a;
  float b;
  float angle;
  float opposite;

  for (int i=0;i<N;i++){
  dist1 += sens1.ping_median(5);
  dist2 += sens2.ping_median(5);
  }
  
  dist1 /= N;
  dist2 /= N; 
   
  distanceXY = (dist1+dist2)/2;
  
  opposite = (dist1-dist2);
  
  if (abs(opposite) < sensWidth)
  {
    angle = asin(opposite/sensWidth)*(180/pi);
    return angle;
  }
  return 100;
}


void positionUpdate(){
  switch(x){
    case 0:
    PORTK = 8 + y;
    break;
    case 1:
    PORTK = 16 + y;
    break;
    case 2:
    PORTK = 16 + y;
    break;
    case 3:
    PORTK = 17 + y;
    break;
    case 4:
    PORTK = 32 + y;
    break;
    case 5:
    PORTK = 33 + y;
    break;
    case 6:
    PORTK = 34 + y;
    break;
    case 7:
    PORTK = 35 + y;
  
  }
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

