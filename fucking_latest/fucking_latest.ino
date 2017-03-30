#include <Adafruit_NeoPixel.h>

#include <gfxfont.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#include <NewPing.h>
#include <stdint.h>

// numerical identities for square qualities
#define unknown 0
#define visited 1
#define dead_end 2
#define obstacle 3
#define foam 5
#define infrastructure 7

// colors
#define RED 0xF800
#define BLUE 0x001F
#define YELLOW 0xFFE0
#define GREEN 0x07E0
#define WHITE 0xffff


byte FWD = B00100010;  //ok
byte REV = B10001000; 
byte rotRight = B00000000; //ok
byte rotLeft = B10101010; //ok
byte strRight = B00001010; //ok
byte strLeft = B10100000; //ok

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
float steps_per_degree = 23.9;
byte motion = B01010101;
float dist_wall;
int period = 1000;



#define Max_Distance   100 
#define arrayIndex 4000       //may have to either decrease or increase the number of sample data reading that we take
 
  int thresHold = 513;          //if analogRead is greater than this value then increase counter
  int thresHoldTick = 600;
  
  int getData[arrayIndex] = {0};
  
  int counter = 0;              //counts of signal going above the threshold
  int micPin= 0;                 //microphone pin number
  int tickPin = 1;
  int infraPin = 3;
  
  int thumperPin = 50;          //thumper pin number
  
  long averaging = 0;
  
  long timerA;
  long timerB;
  
  int32_t middle = 0;


//********** sensor variables

    // Define various ADC prescaler
    const unsigned char PS_16 = (1 << ADPS2);
    const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
    const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
    const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

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
  
  float distanceToMove;
  
  float oneFoot = 12/micro_to_inches;
  float Ycentered = 0.75/micro_to_inches;
  float Xcentered = 1.25/micro_to_inches;
  
  int Yoffset;
  int Xoffset;
  int yTolerance = 40; //microseconds
  
  Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8,8,52,
    NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT +
    NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
    NEO_GRB + NEO_KHZ800);

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
                  
  

//**************************************************SETUP
void setup() 
{
  Serial.begin(9600);
  byte allOutputs = B11111111;
  matrix.begin();
  DDRL = allOutputs;
  DDRB = allOutputs;
  DDRK = allOutputs;
  
  analogReference(INTERNAL2V56);
  
  for(int i=0;i<5;i++){
  analogRead(i);
  }
  
  PORTK = 0x00;
  PORTB = 0x00;
  
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_64;
  
  pinMode(7,OUTPUT);
  pinMode(6,OUTPUT);
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_64;
    

x = 0;
y = 0;

/*

matrix.setBrightness(10);
matrix.drawPixel(x,y,BLUE);
matrix.show();

powerTick();

for(int i=0;i<10;i++){
  Serial.println(thumpTick());
}
tickCalibrate();
Serial.print("baseline");
Serial.println(middle);

matrix.drawPixel(3,3,0x0000);
matrix.show();

delay(5000);
*/

//radialSearch();
//turn(360*2,1,period);


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

//gridSearch_with_strafe();
}

void loop() 
{
//Serial.println(checkFrontClear());
//Serial.println(checkLeftBlock());
//tickTracer();
//thump();
/*Serial.println("FRONT");
angleMeas(sonarFL, sonarFR, frontWidth);
delay(100);
Serial.println("LEFT");
angleMeas(sonarLB, sonarLF, sideWidth);
delay(100);
Serial.println("BACK");
angleMeas(sonarBR, sonarBL, frontWidth);
delay(100);
Serial.println("RIGHT");
angleMeas(sonarRF, sonarRB, sideWidth);
delay(100);*/
transCorrection();
}

/*void radialSearch()
{
  
  // x dimension positive
  while(x<5){
    // move into next x square
    ForwardBackward(12,1,period);
    x++;
    matrix.drawPixel(x,y,0xffff);
    matrix.show();
    transCorrection(x);
    // if no obstacle in (1,1)
    if(checkLeftBlock()){
      board[x][y+1] = obstacle;
    }
    // no obstacle
    else if(board[x][y+1] <= visited){
      //strafe(0.5,1,period);
      turn(90,0,period);
      rotateCorrection();
      int spokeLength = 0;
      //ForwardBackward(0.5,0,period);
      while(checkFrontClear() && y<5 && yVisited()){ // make visit check smarter
        ForwardBackward(12,1,period);
        y++;
//        matrix.drawPixel(x,y,0xffff);
//        matrix.show();
        spokeLength++;
        rotateCorrection();
        //spokeCorrection(1);
        board[x][y] = thumpTick();
      }
        board[x][y+1] = obstacle;
      for(int i=spokeLength;i>0;i--){
        
        ForwardBackward(12,0,period);
        rotateCorrection();
        y--;
        //spokeCorrection(1);
        
      }
     
      rotateCorrection();
      turn(90,1,period);
      
    }
  }
  ForwardBackward(12,1,period);
  x++;
  matrix.drawPixel(x,y,0xffff);
  matrix.show();
  //transCorrection(x);
  rotateCorrection();
  turn(90,0,period);
  rotateCorrection();
  while(y<5){
    ForwardBackward(12,1,period);
    y++;
//    matrix.drawPixel(x,y,0xffff);
//    matrix.show();
    transCorrection(y);
    if(checkLeftBlock()){
      board[x-1][y] = obstacle;
    }
    else if(board[x-1][y] <= visited){
      strafe(0.5,1,period);
      turn(90,0,period);
      rotateCorrection();
      int spokeLength = 0;
      ForwardBackward(0.5,0,period);
      while(checkFrontClear() && x!=1 && xVisited()){ // smarter function
        ForwardBackward(12,1,period);
        x--;
        spokeLength++;
        rotateCorrection();
        spokeCorrection(spokeLength);
        board[x][y] = thumpTick();
      }
      board[x-1][y] = obstacle;
      for(int i=0;i<spokeLength;i++){
        ForwardBackward(12,0,period);
        x++;
        rotateCorrection();
        spokeCorrection(spokeLength);
       
      }
      spokeLength = 0;
      rotateCorrection();
      turn(90,1,period);
    }
  }
  ForwardBackward(12,1,period);
  y++;
  matrix.drawPixel(x,y,0xffff);
  matrix.show();
  transCorrection(x);
  rotateCorrection();
  turn(90,0,period);
  rotateCorrection();
  
  while(x>1){
    ForwardBackward(12,1,period);
    x--;
//    matrix.drawPixel(x,y,0xffff);
//    matrix.show();
    transCorrection(6-x);
    if(checkLeftBlock()){
      board[x][y-1] = obstacle;
    }
    else if(board[x][y-1] <= visited){
      strafe(0.5,1,period);
      turn(90,0,period);
      rotateCorrection();
      int spokeLength = 0;
      ForwardBackward(0.5,0,period);
      while(checkFrontClear() && y!=1 && yVisited()){ // smarter function
        ForwardBackward(12,1,period);
        y--;
//        matrix.drawPixel(x,y,0xffff);
//        matrix.show();
        spokeLength++;
        spokeCorrection(spokeLength);
        rotateCorrection();
        board[x][y] = thumpTick();
       
      }
        board[x][y-1] = obstacle;
      for(int i=0;i<spokeLength;i++){
        ForwardBackward(12,0,period);
         y++;
         spokeCorrection(spokeLength);
       }
      rotateCorrection();
      turn(90,1,period);
    }
  }
  ForwardBackward(12,1,period);
  x--;
  matrix.drawPixel(x,y,0xffff);
  matrix.show();
  transCorrection(x);
  rotateCorrection();
  turn(90,0,period);
  rotateCorrection();
  while(y>1){
    ForwardBackward(12,1,period);
    y--;
//    matrix.drawPixel(x,y,0xffffff);
//    matrix.show();
    transCorrection(6-y);
    if(checkLeftBlock()){
      board[x+1][y] = obstacle;
    }
    else if(board[x+1][y] <= visited){
      strafe(0.5,1,period);
      turn(90,0,period);
      rotateCorrection();
      ForwardBackward(0.5,0,period);
      int spokeLength = 0;
      while(checkFrontClear() && x!=5 && xVisited()){ // smarter function
        ForwardBackward(12,1,period);
        x++;
        spokeLength++;
        rotateCorrection();
        spokeCorrection(spokeLength);
        board[x][y] = thumpTick();
        
      }
      board[x+1][y] = obstacle;
      for(int i=0;i<spokeLength;i++){
        ForwardBackward(12,0,period);
        x--;
        rotateCorrection();
        spokeCorrection(spokeLength);
      }
      rotateCorrection();
      turn(90,1,period);
    }
  }  
  ForwardBackward(12,1,period);
  y--;
//  matrix.drawPixel(x,y,0xffffff);
//  matrix.show();
  transCorrection();
  rotateCorrection();
  turn(90,0,period);
  rotateCorrection();
}





        */

/*void gridSearch()
{
matrix.clear();  
ForwardBackward(12,1,period);
x++;
transCorrection(x);
turn(90,0,period);
rotateCorrection();
ForwardBackward(12,1,period);
y++;
transCorrection(y);


board[x][y] = thumpTick();
//markHere(x,y);
while ((x < 5) || (y < 5))
  {
    if ( (x % 2) != 0 && y < 5)
        {
          ForwardBackward(12,1,period);
          y++;
          board[x][y] = thumpTick();
          if(y==3){
            rotateCorrection();
          }
          //check tick tracer / dead end
          //display result
          //markHere(x,y);
        }
    else if ( (x % 2) == 0 && y > 1)
    {
      ForwardBackward(12,0,period);
      y--;
      if(y==3){
            rotateCorrection();
          }
          board[x][y] = thumpTick();
          //display result
          //markHere(x,y);
    }
    else if (y == 5 && x%2 != 0)
      {
        transCorrection(y);
        turn(90,1,period);
        rotateCorrection();
        transCorrection(x); 
        //transCorrection(x);     
        ForwardBackward(12,1,period);
        x++;
        turn(90,0,period);
        rotateCorrection();
        transCorrection(y);
        
        board[x][y] = thumpTick();
          //check tick tracer / dead end
          //display result
          //markHere(x,y);
      }
    else if (y == 1 && x%2==0)
      {
      transCorrection(y);
      turn(90,1,period);
      rotateCorrection();
      transCorrection(x);
      //transCorrection(x); 
      ForwardBackward(12,1,period);
      x++;
      turn(90,0,period);
      rotateCorrection();
      transCorrection(y);
      
      board[x][y] = thumpTick();
      //check tick tracer / dead end
      //display result
      //markHere(x,y);
      }  
   }
}


*/
void gridSearch_with_strafe()
{
x=0;
y=0;
matrix.clear();  
strafe(12,1,period);
x++;
ForwardBackward(12,1,period);

y++;            //we are in [1,1]
transCorrection();

//board[x][y] = thumpTick();
//markHere(x,y);
while ((x < 5) || (y < 5))
  {
    if ( (x % 2) != 0 && y < 5)
        {
          ForwardBackward(12,1,period);
          y++;
          //board[x][y] = thumpTick();
          /*if(y==3){
            rotateCorrection();
          }
         */
          //check tick tracer / dead end
          //display result
          //markHere(x,y);
        }
    else if ( (x % 2) == 0 && y > 1)
    {
      ForwardBackward(12,0,period);
      y--;
   
          //board[x][y] = thumpTick();
          //display result
          //markHere(x,y);
    }
    else if (y == 5 && x%2 != 0)
      {
       
        
       //rotateCorrection();
       strafe(12,1,period);
       transCorrection();
       
        //transCorrection(x);     
  
        x++;
   
        
        //board[x][y] = thumpTick();
          //check tick tracer / dead end
          //display result
          //markHere(x,y);
      }
    else if (y == 1 && x%2==0)
      {
        
//transCorrection();
       // rotateCorrection();
    strafe(12,1,period);
      //transCorrection(x); 
     transCorrection();
      x++;

      
      //board[x][y] = thumpTick();
      //check tick tracer / dead end
      //display result
      //markHere(x,y);
      }  
  }
}

// BOARD LOGIC
/////////////////////////////////////////////////////////////////////////
boolean xVisited(){
  for(int i=1;i<=5;i++){
    if (board[i][y] == 0)
    {
      return false;
    }
  }
  return true;
}

boolean yVisited(){
  for(int i=1;i<=5;i++){
    if (board[x][i] == 0)
    {
      return false;
    }
  }
  return true;
}



//&&&&&&&&&&&&&&&&& thumper and tick tracer &&&&&&&&&&&&&&&&&&&&&&&&&

int thumpTick(){
    delay(100);
    int decision;
    PORTB ^= 0x08;
    delay(20);
    int counter = 0;
    //NEED TO ADD BASELINE MEASUREMENT
    for (int i =0; i<arrayIndex;i++){ 
      int value = analogRead(micPin);
      if(value > thresHold){
          counter ++;
        }
    }
    Serial.println(counter);
      if(counter > 3880 && counter < 3890){
          Serial.println("Hollow");
          decision = dead_end;
          matrix.drawPixel(x,y,BLUE);
          matrix.show();
        }else if(counter > 3920){
            Serial.println("Solid");
            decision = foam;
          }else{
             Serial.println("Wire");
             decision = infrastructure ;
             matrix.drawPixel(x,y,GREEN);
             matrix.show();
            }

     delay(50);
    PORTB ^= 0x08; //Down
    //delay(24)
  
    
     //timerB = micros();
     delay(200);
  
    int32_t value = 0;
    int32_t acceptable = 50;
    for (int i =0; i<arrayIndex;i++){ 
      value += analogRead(tickPin);
    }
   //instead of serialprintln, print directly onto the board
   
   
   
   
    value /= arrayIndex;
    Serial.println(value);
    delay(100);
    Serial.println(abs(middle-value));
    
    if (abs(middle-value) < acceptable)
    {
      Serial.println("wire");
      decision = infrastructure;
      matrix.drawPixel(x,y,RED);
      matrix.show();
    }


     delay(200);
     
//     Serial.print("Counter: ");
//     Serial.println(counter);
 

    
    return decision;
}

void tickCalibrate(){

    matrix.drawPixel(3,3,WHITE);
    matrix.show();
  
    int32_t value = 0;
    
    for (int i =0; i<arrayIndex;i++){ 
      value += analogRead(tickPin);
    }
    value /= arrayIndex;
    
    middle = value;

     delay(200);

}


void powerTick(){
    delay(2000);
    digitalWrite(51,HIGH);
    delay(500);
    digitalWrite(51,LOW);
    delay(2000);
    digitalWrite(51,HIGH);
    delay(2500);
    digitalWrite(51,LOW);
}

void rotateCorrection()
{
  
  float angleFront;
  float angleLeft;
  float angleBack;
  float angleRight;

  float minMag;
   


  if ( y < 3)
  {  
    if ( x < 3)
    {
      angleLeft = angleMeas(sonarLB, sonarLF, sideWidth);
      minMag = (angleLeft+angleBack)/2; //quadrant 3
    }
    else       
    {
      angleRight = angleMeas(sonarRF, sonarRB, sideWidth);
      minMag = (angleRight+angleBack)/2; //quadrant 4
    }
  }
  else 
  { 
    if (x < 3)
    {
      angleLeft = angleMeas(sonarLB, sonarLF, sideWidth);
      minMag = (angleLeft+angleFront)/2; //quadrant 2
    }
    else
    {
      angleRight = angleMeas(sonarRF, sonarRB, sideWidth);
      minMag = (angleRight+angleFront)/2; //quadrant 1
    }
  }
  
  if (minMag>0.75)
  {
    turn(minMag, 1, period);
    digitalWrite(7,HIGH);
  }
  else if (minMag < -0.75)
  {
    turn(abs(minMag), 0, period);
    digitalWrite(7,HIGH);
  }
  digitalWrite(7,LOW);
}

//****************** align to wall
void transCorrection()
{
  float backDistance;
  float frontDistance;

  float leftDistance;
  float rightDistance;
  if (y < 3)
  {
  angleMeas(sonarBR, sonarBL, frontWidth); 
  backDistance = distanceXY; 
  Yoffset = (backDistance-Ycentered)-oneFoot*(y);
  Serial.println("y distance correction :");
  Serial.println(Yoffset*micro_to_inches);
  }
  else
  {
  angleMeas(sonarFL, sonarFR, frontWidth);
  frontDistance = distanceXY;
  Yoffset = -((frontDistance-Ycentered)-oneFoot*(6-y));
    Serial.println("y distance correction :");
  Serial.println(Yoffset*micro_to_inches);
  }  
 //********** new 
  if (x < 3)
  {
  angleMeas(sonarLB, sonarLF, sideWidth); 
  leftDistance = distanceXY; 
  Xoffset = (leftDistance-Xcentered)-oneFoot*(x);
    Serial.println("x distance correction :");
  Serial.println(Xoffset*micro_to_inches);
  }
  else
  {
  angleMeas(sonarRB, sonarRF, sideWidth);
  rightDistance = distanceXY;
  Xoffset = -((rightDistance-Xcentered)-oneFoot*(6-x));
    Serial.println("x distance correction :");
  Serial.println(Xoffset*micro_to_inches);
  }
  
  // y correction

  
  if (abs(Yoffset) > 18){
     if (Yoffset > 0)
    
      ForwardBackward(Yoffset*micro_to_inches,0,period);
    
    else
        ForwardBackward(abs(Yoffset*micro_to_inches),1,period);
    
  }
//x correction

  if (abs(Xoffset) > 18){
     if (Xoffset > 0)
    {
      strafe(Xoffset*micro_to_inches,0,period);
    }
    else
    {
      strafe(abs(Xoffset*micro_to_inches),1,period);
    }
  }
}

void spokeCorrection(int count)
{
  float backDistance;

  angleMeas(sonarBR, sonarBL, frontWidth); 
  backDistance = distanceXY; 
  Yoffset = (backDistance-Ycentered)-oneFoot*(count);

 
  if (abs(Yoffset) > 18){
     if (Yoffset > 0)
    {
      ForwardBackward(Yoffset*micro_to_inches,0,period);
    }
    else
    {
      ForwardBackward(abs(Yoffset*micro_to_inches),1,period);
    }
  }
}

boolean checkLeftBlock(){
  
      int N;
//    float reflectanceAdjust = 1.0;
//    //float maximum = 1.2;
//    float volt_to_dig = 2.65/1024;
//    float value = 0;
//    
//    for (int i =0; i<N;i++){ 
//      value += analogRead(2)*volt_to_dig;
//     }
//     
//     value /= (float)N;
//     Serial.println(value);
     
      N = 3;
      float dist1 = 0;
      float dist2 = 0;
      
      int pingMax = 500;
    
      for (int i=0;i<N;i++){
      dist1 += sonarLB.ping_median(5);
      dist2 += sonarLF.ping_median(5);
      }
      
      dist1 /= N;
      dist2 /= N; 
       
      distanceXY = (int)max(dist1,dist2);
    
      
      Serial.print("left: ");
      Serial.println(dist1);
      Serial.print("right: ");
      Serial.println(dist2);
      Serial.print("avg: ");
      Serial.println(distanceXY);
     
    
  if(distanceXY > pingMax || distanceXY == -1 || distanceXY == 0){
    return false;
  }
  
    return true;

}

boolean checkFrontClear(){
  
    int N = 1000;
    float reflectanceAdjust = 0.6;
    float volt_to_dig = 2.65/1024;
    float value = 0;
    
    for (int i =0; i<N;i++){ 
      value += analogRead(infraPin)*volt_to_dig;
     }
     
     value /= (float)N;
     Serial.println(value);
     
      N = 3;
      float dist1 = 0;
      float dist2 = 0;
      
      for (int i=0;i<N;i++){
      dist1 += sonarFL.ping_median(5);
      dist2 += sonarFR.ping_median(5);
      }
      
      dist1 /= N;
      dist2 /= N; 
       
      distanceXY = (int)max(dist1,dist2);
    
      Serial.print("left: ");
      Serial.println(dist1);
      Serial.print("right: ");
      Serial.println(dist2);
      Serial.print("avg: ");
      Serial.println(distanceXY);
  
  
    
  if(value < reflectanceAdjust || distanceXY == -1 || distanceXY > 300){
    return true;
  }
  
    return false;
  
}
    
//**********************************************************Calculate angle
// firts sensor is MOST COUNTERCLOCKWISE
float angleMeas(NewPing sens1, NewPing sens2, float sensWidth)//sensWidth front  = 6.75, sensWidth sides = 7.0625
{
  int N = 3;
  float dist1 = 0;
  float dist2 = 0;
  float angle;
  float opposite;

  for (int i=0;i<N;i++){
  dist1 += sens1.ping_median(5);
  dist2 += sens2.ping_median(5);
  }
  
  dist1 /= N;
  dist2 /= N; 
   
  distanceXY = (dist1+dist2)/2;

  
  Serial.print("FIRST: ");
  Serial.println(dist1*micro_to_inches);
  Serial.print("SECOND: ");
  Serial.println(dist2*micro_to_inches);
  Serial.print("avg: ");
  Serial.println(distanceXY*micro_to_inches);
  
  opposite = (dist1-dist2);
  
  if (abs(opposite) < sensWidth)
  {
    angle = asin(opposite/sensWidth)*(180/pi);
    return angle;
  }
  return 0;
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

