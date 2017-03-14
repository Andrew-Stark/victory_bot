
  #include <NewPing.h>
  #include <Adafruit_LEDBackpack.h>

  #define TRIGGER_Sensor1  34  // Arduino pin tied to trigger pin on the firts ultrasonic sensor.

  #define ECHO_Sensor1     35  // Arduino pin tied to echo pin on the first ultrasonic sensor.

  #define TRIGGER_Sensor2  30  // Arduino pin tied to trigger pin on the second ultrasonic sensor.

  #define ECHO_Sensor2     31  // Arduino pin tied to echo pin on the second ultrasonic sensor.

  #define TRIGGER_Sensor3  22  // Arduino pin tied to trigger pin on the third ultrasonic sensor.

  #define ECHO_Sensor3     23  // Arduino pin tied to echo pin on the third ultrasonic sensor.

  #define TRIGGER_Sensor4  26  // Arduino pin tied to trigger pin on the fourth ultrasonic sensor.

  #define ECHO_Sensor4     27  // Arduino pin tied to echo pin on the fourth ultrasonic sensor.

  #define TRIGGER_Sensor5  28  // Arduino pin tied to trigger pin on the fifth ultrasonic sensor.

  #define ECHO_Sensor5     29  // Arduino pin tied to echo pin on the fifth ultrasonic sensor.

  #define TRIGGER_Sensor6  36  // Arduino pin tied to trigger pin on the sixth ultrasonic sensor.

  #define ECHO_Sensor6     37  // Arduino pin tied to echo pin on the sixth ultrasonic sensor.

  #define TRIGGER_Sensor7  32  // Arduino pin tied to trigger pin on the seventh ultrasonic sensor.

  #define ECHO_Sensor7     33  // Arduino pin tied to echo pin on the seventh ultrasonic sensor.

  #define TRIGGER_Sensor8  24  // Arduino pin tied to trigger pin on the eighth ultrasonic sensor.

  #define ECHO_Sensor8     25  // Arduino pin tied to echo pin on the eighth ultrasonic sensor.


  #define MAX_DISTANCE    200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

  NewPing LF(TRIGGER_Sensor1, ECHO_Sensor1, MAX_DISTANCE); // NewPing setup of pins and maximum distance for first sensor. //left front
  NewPing LR(TRIGGER_Sensor2, ECHO_Sensor2, MAX_DISTANCE); // NewPing setup of pins and maximum distance for second sensor. // left rear
  NewPing RR(TRIGGER_Sensor3, ECHO_Sensor3, MAX_DISTANCE); // NewPing setup of pins and maximum distance for third sensor.  //right rear
  NewPing RF(TRIGGER_Sensor4, ECHO_Sensor4, MAX_DISTANCE); // NewPing setup of pins and maximum distance for fourth sensor. //right front
  NewPing FR(TRIGGER_Sensor5, ECHO_Sensor5, MAX_DISTANCE); // NewPing setup of pins and maximum distance for fifth sensor.  // front right
  NewPing FL(TRIGGER_Sensor6, ECHO_Sensor6, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sixth sensor.  //front left
  NewPing RL(TRIGGER_Sensor7, ECHO_Sensor7, MAX_DISTANCE); // NewPing setup of pins and maximum distance for seventh sensor. //rear left
  NewPing ReR(TRIGGER_Sensor8, ECHO_Sensor8, MAX_DISTANCE); // NewPing setup of pins and maximum distance for eighth sensor. //rear right
  
  //RL FR RR FL
  float pi = 3.14159265359;
  float distance = 60*pi/25.4;
  byte fwd = B00100010;
  byte rev = B10001000;
  byte rotRight = B00000000;
  byte rotLeft = B10101010;
  byte StrLeft = B10000010;
  byte StrRight = B00101000;
  
  byte motionrear = B00010001;//Rear
  byte motionfront = B01000100;//front
  byte motion45Left = B00000101; // motion rightwheels
  byte motion45Right = B01010000;
  byte motionRightWheels = B00010001;// RIGHT WHEELS
  byte motionLeftWheels = B01000100;// LEFT WHEELS
  byte mov = B01010101;
  float torDist = .2;
  float Steps_per_inch = 215.6;
  float spd = 23.8;//steps per degree
  float distanceWall;
  float magnitude; //magnitude return by measureAngle
  float leftWall;
  float rightWall;
  float frontWall;
  float rearWall;
  int 0;
  int 0;
  int board[7][7]= {{0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0}};
                    
                     
  void moving(float, int);
  void motion45d(float,byte,byte);
  float measureAngle(int, NewPing, NewPing);
  void rotate(float, byte);
  void avoidMe(int, int);
  void case1();
  void case2();
  Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();
  
  
void setup() {
  // put your setup code here, to run once:
  byte allOutputs = B11111111;
  Serial.begin(9600);
/* 
  PORTL = fwd; 
  for(int i = 0; i < 300000; i++){
  PORTL ^= mov;
  delayMicroseconds(500);
  }
  */
//  moving(12.0,fwd);
//  delay(500);
//  moving(12.0,rev);
//  delay(500);
//  moving(12.0,rotRight);
//  delay(500);
//  moving(12.0,rotLeft);
//  delay(500);
//  moving(24.0,StrRight);
//  delay(500);
//  moving(24.0,StrLeft);
//  matrix.begin(0x70);
//  markHere(0,0);
//  delay(500);
//    markHere(0,7);
//  delay(500);
//    markHere(7,0);
//  delay(500);
//    markHere(7,7);
//  delay(500);
//    markHere(3,3);
//  delay(500);
//    markHere(3,4);
//  delay(500);
//    markHere(4,3);
//  delay(500);
//    markHere(4,4);
//  delay(500);
  gridSearch();
/*
 * moving(float inch, byte dir)
 * rotate(float degree, byte direct)
 * motion45d (float inch, byte direct, byte side)
 * arcMotion (float arcLength, byte direct, byte side)
 * pivotMotion (float inch, byte direct, byte br)
 * speed_variable(float inch, int dlay, float speed_ratio, byte d1, byte d2)
 * accel( byte d1,byte d2,float speed_ratio, float in,float lay, int N)
 * measureAngle(5, sensor1(Left or front), sensor2(Right or rear));
 */

//moving((12.0 * 1.2), StrLeft);
//align();
//rotate(90.0, rotRight);
//motion45d(36.0,fwd,motion45Left);
//arcMotion (12.0, fwd, motionRightWheels);
//pivotMotion(48.0, rotRight, motionrear);
//speed_variable(24.0, 500, 2.0 ,motionRightWheels, motionLeftWheels);
//accel(motionRightWheels, motionLeftWheels, 1, 2.0, 800, 16);
//runAlong(60.0);

//gridSearch();
//align();
}

void loop() {
//put your main code here, to run repeatedly:

//Serial.print("Left Front = ");
//Serial.println(LF.ping_median(5));
//
//Serial.print(" Right Front = ");
//Serial.println(RF.ping_median(5));
//
//Serial.print(" Left Rear = ");
//Serial.println(LR.ping_median(5));
//
//Serial.print(" Right Rear = ");
//Serial.println(RR.ping_median(5));
//
//Serial.print(" Front Left = ");
//Serial.println(FL.ping_median(5));
//
//Serial.print(" Front Right = ");
//Serial.println(FR.ping_median(5));
//
//Serial.print(" Rear Left = ");
//Serial.println(RL.ping_median(5));
//
//Serial.print(" Rear Right = ");
//Serial.println(ReR.ping_median(5));

/*
Serial.print("Left Angle   =   ");
Serial.print(measureAngle(8, LF, LR));
Serial.print(" Distance to wall  =   "); 
Serial.println(distanceWall);
delay(100);
*//*
Serial.print(" Left: ");
Serial.print(measureAngleSide(5, LF, LR));

Serial.print(" Right: ");
Serial.print(measureAngleSide(5, RF, RR));

Serial.print(" Front: ");
Serial.print(measureAngleBF(5, FL, FR));

Serial.print(" Rear: ");
Serial.println(measureAngleBF(5, RL, ReR));
*/

//align();
} 

void displayLED(){
  
  for(int i = 0; i <=7; i++){
    for(int j = 0; j <=7; j++){
      matrix.drawPixel(i,j, LED_GREEN);
      matrix.writeDisplay();  
    }//end of second for
  }//end of first for
}
void markHere(int x, int y){
  matrix.drawPixel(x,y, LED_RED);
  matrix.writeDisplay();
}//end of marking the wire

void gridSearch(){
  
  motion45d(12.0, fwd, motion45Right);
  x = 1;
  y = 1;
  align();
  delayMicroseconds(1000);
  //align();
  while( (y < 5) ||  (x < 5)){
    if( (y % 2 == 1)&& (x <5)){
      //measureAngle(5,FR , FL);
      //if(distanceWall < 10){
        //board[x+1][y] = 1;
        //checkMe(x,y);
        //avoidMe(x,y);
      //}else{
        moving(12.0, fwd);
        x++;
      //}
    }else if( (y % 2 == 0) && (x > 1)){
      //measureAngle(5, ReR, RL);
      //if(distanceWall < 10){
        //board[x-1][y] =1;
        //checkMe(x,y);
        //avoidMe(x,y);
      //}else{
        moving(12.0, rev);
        x--;
      //}
    }else if(((y % 2 == 1) && (x == 5)) || ((y % 2 == 0) && (x ==1))){
      //measureAngle(5, RR, RF);
      //if(distanceWall < 10){
        //board[x][y+1] = 1;
        //checkMe(x,y);
        //avoidMe(x,y);
      //}else{
        moving(12.0 , StrRight);
        align();
        delayMicroseconds(1000);
        //align();
        y++; 
      //}
    }//end of else if
  }//end of while
}//end of gridsearch

 void goHome(){
  moving(12.0, fwd);
  moving(60.0, StrLeft);
  moving(72.0, rev);
 }

void checkMe(int x, int y){
  measureAngleSide(5, RR, RF);
  int nextRight = distanceWall;
  measureAngleSide(5,LR, LF);
  int nextLeft = distanceWall;
 
  if(nextRight < 10){
    board[x][y+1] = 1;
  }else if(nextLeft < 10){
    board[x][y-1] = 1;
  }else if ((nextLeft < 10) && (nextRight <10)){
    board[x][y+1] = 1;
    board[x][y-1] = 1;
  }
}
void avoidMe(int x, int y){
  if((y % 2 == 1) && (x <= 3)){
    case1();
  }else if((y % 2 == 0) && (x >= 3)){
    case2();
  }else if (((y % 2 == 1) && (x == 4)) && (y != 5)){
    case3();
  }else if (((y % 2 == 1) && (x ==5)) && (y != 5)){
    case4();
  }else if ((y % 2 == 0) && (x == 2)){
    case5();
  }else if((y % 2 == 0) && (x == 1)){
    case6();
  }else if ((y == 5) && (x == 4)){
    //case7();
  }
  
}
void case1(){
  int l = board[x][y-1] + board[x+1][y-1] + board[x+2][y-1];

  if(l > 0){
    moving(12.0 , StrLeft);
    moving(24.0, fwd);
    moving(12.0, StrRight);
  }else{
    moving(12.0 , StrRight);
    moving(24.0, fwd);
    moving(12.0, StrLeft);
  }
  x = x +2; 
}
void case2(){
  int p = board[x][y+1] + board[x-1][y+1] + board[x-2][y+1];
  if(p > 0){
    moving(12.0 , StrRight);
    moving(24.0,rev);
    moving(12.0, StrLeft);
  }else{
    moving(12.0 , StrLeft);
    moving(24.0, rev);
    moving(12.0, StrRight);
  }
  x = x -2;   
}

void case3(){
int p = board[x][y-1] + board[x+1][y-1] + board[x+2][y-1];
  if(p > 0){
    moving(12.0, StrRight);
    moving(12.0, fwd);
     
  }else{
    moving(12.0, StrLeft);
    moving(24.0, fwd);
    moving(24.0, StrRight);
    moving(12.0, rev);
  }
}//end case 3

void case4(){
  int p = board[x-1][y];
  if(p>0){
    moving(12.0,StrLeft);
    moving(24.0, rev);
    moving(24.0, StrRight);
    moving(12.0, fwd);
  }else{
    moving(12.0, rev);
    moving(12.0, StrRight);
  }
}//end case 4

void case5(){
  int p = board[x][y-1] + board[x-1][y-1];
  if(p>0){
    moving(12.0, fwd);
    moving(24.0, StrLeft);
    moving(32.0, rev);
    moving(32.0, StrRight);
    moving(12.0, fwd);
  }else{
    moving(12.0, StrLeft);
    moving(24.0, rev);
    moving(24.0, StrRight);
    moving(12.0, fwd);
  }
  
}//end of case 5

void case6(){
  int p = board[x+1][y];
  if(p>0){
    moving(12.0,StrLeft);
    moving(24.0, fwd);
    moving(24.0, StrRight);
    moving(12.0, rev);
  }else{
    moving(12.0, fwd);
    moving(12.0, StrRight);
  }
}//end of case 6
/*
void case7(){
  
}
*/

//currently only for wall on the right
void runAlong(float inch,int wall){
  float d;
  float dwall1;
  float dwall2;
  float cAngle;
  
  if(wall == 1){
  align();
  dwall1 = distanceWall;
  while(inch >0){
    
    moving(12.0,fwd);
    align();
    dwall2 = distanceWall;
    d = dwall2 - dwall1;
    cAngle = atan(d/12) * 180/pi;
    if(cAngle > 0){
    rotate(cAngle,rotLeft);
    }else{
      rotate(abs(cAngle),rotRight);
    }
      inch -= 12.0;
    }
  }//end of leftWall
  else if (wall == 0){
      align();
      dwall1 = distanceWall;
   while(inch >0){
    moving(12.0,fwd);
    align();
    dwall2 = distanceWall;
    d = dwall2 - dwall1;
    cAngle = atan(d/12) * 180/pi;
    if(cAngle > 0){
    rotate(cAngle,rotRight);
    }else{
      rotate(abs(cAngle),rotLeft);
    }
      inch -= 12.0;
    } 
  }//end of rightWall
  else{
  }
}

//LF - LR
float measureAngleSide(int N, NewPing sensor1, NewPing sensor2 ){
  float angle=0;
  float a;
  float b;
  float c=0;
   for(int i=0; i< N; i++){
    a = sensor1.ping_median();
    b = sensor2.ping_median();
    c += (a + b)/2;
    angle += (a - b) * .006756;
     
   }
   c /= N;
   angle /= N;
   angle = asin(angle/8.0) * 180/pi; // 7.5 is the distance between the two sensor
   distanceWall = c * .006756;

   return angle;
    
}
float measureAngleBF(int N, NewPing sensor1, NewPing sensor2 ){
  float angle=0;
  float a;
  float b;
  float c=0;
   for(int i=0; i< N; i++){
    a = sensor1.ping_median();
    b = sensor2.ping_median();
    c += (a + b)/2;
    angle += (a - b) * .006756;
     
   }
   c /= N;
   angle /= N;
   angle = asin(angle/4.0)* 180/pi; // 7.5 is the distance between the two sensor
   distanceWall = c * .006756;

   return angle;
    
}


void align(){
  float angleL;
  float angleR;
  float angleF;
  float angleRear;
  float minMag;
  angleL = measureAngleSide(5, LF, LR);
  leftWall = distanceWall;
  angleR = measureAngleSide(5,RF,RR);
  rightWall = distanceWall;
  angleF = measureAngleBF(5,FL, FR);
  frontWall = distanceWall;
  angleRear = measureAngleBF(5,RL, ReR);
  rearWall = distanceWall;
  Serial.print(" Left: ");
  Serial.print(angleL);

  Serial.print(" Right: ");
  Serial.print(angleR);

  Serial.print(" Front: ");
  Serial.print(angleF);

  Serial.print(" Rear: ");
  Serial.println(angleRear);
 if(angleL != angleL){
  minMag = 100; 
 }else{
    minMag = angleL;
 }
  if ((abs(minMag) > abs(angleR)) && !(angleR != angleR))
    minMag = angleR;
  else if((abs(minMag) > abs(angleF)) && !(angleF != angleF))
    minMag = angleF;
  else if((abs(minMag) > abs(angleRear))  && !(angleR != angleR))
    minMag = angleRear;
 if(minMag == 100){
  minMag = angleRear;
 }
Serial.println(minMag);

if(((minMag > 0) && ( angleL < 0)) || ((minMag <0 ) && (angleL < 0)) || ((minMag > 0) && (angleR >0))) {
  rotate(abs(minMag), rotRight);
}
 else if (((minMag < 0) && (angleL >0)) || ((minMag > 0 ) && (angleL > 0)) || ((minMag < 0) && (angleR < 0))) {
  rotate(abs(minMag), rotLeft);
 }
     
}//end of align




void moving(float inch, byte dir){
  float steps_f = inch * Steps_per_inch;
  int steps = steps_f;

    PORTL = dir;
    for (int i = 0; i < steps; i++){
      PORTL ^= mov;
      delayMicroseconds(500);
    }
}//end of reverse function



void rotate(float degree, byte direct){
  float steps_per_degree = spd * degree;
  
  int steps = steps_per_degree;
  PORTL = direct;
  for(int i = 0; i < steps; i++){
    PORTL ^= mov;
    delayMicroseconds(500);
  }
}//end of function for rotateRight



void motion45d (float inch, byte direct, byte side ){
  
  float steps_f = inch * Steps_per_inch;
  long steps = steps_f * 2;

    PORTL = direct;
    for (long i = 0; i < steps; i++){
      PORTL ^= side;
      delayMicroseconds(500);
    }  
}//end of motion45d

void arcMotion (float arcLength, byte direct, byte side){
  
   float steps_f = arcLength * Steps_per_inch;
   long steps = steps_f;

    PORTL = direct;
    for (long i = 0; i < steps; i++){
      PORTL ^= side;
      delayMicroseconds(2000);
    }  
}//end of function arcMotion

void pivotMotion (float inch, byte direct, byte br){
  
  float steps_f = inch * Steps_per_inch;
  long steps = steps_f;

    PORTL = direct;
    for (long i = 0; i < steps; i++){
      PORTL ^= br;
      delayMicroseconds(2000);
    }  
}


void speed_variable(float inch, int dlay, float speed_ratio, byte d1, byte d2){
  int delayMaster = dlay;
  int delaySlave = dlay * speed_ratio;
  int stepMaster = 0;
  int stepSlave = 0;
  float steps_f = inch * Steps_per_inch;
  long steps = steps_f;
   PORTL = fwd;
   for( int i = 0; i < steps;){
    if (stepMaster >= delayMaster){
          PORTL ^= d1;
          stepMaster = 0;
          i++;
    }
    if (stepSlave >= delaySlave){
        PORTL ^= d2;
        stepSlave = 0;
    }
    stepMaster++;
    stepSlave++;
   }
}//end of speed_variabel

void accel( byte d1,byte d2,float speed_ratio, float in,float lay, int N){
    float go;
   int start;
  for( long i = 1; i <= N; i++){
    go = torDist * i;
    start = lay / i;
    speed_variable(go, start, speed_ratio, d1, d2 ); 
  }
  
  go = in - (N*(N+1)*torDist);
  speed_variable(go, start, speed_ratio, d1, d2);
  
  for(long i = N; i >= 1; i--){
    go = torDist * i;
    start = lay / i;
    speed_variable(go, start, speed_ratio, d1, d2);
    
  }
}



