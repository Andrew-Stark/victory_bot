   #define arrayIndex 1000  
  int thresHold = 600;                                  
  int micPin= 0;                 //microphone pin number
  int thumperPin = 50;          //thumper pin number
  long timerA;
  long timerB;



void setup() {
  Serial.begin(115200);
  byte allOutputs = B11111111;
  DDRL = allOutputs;
  DDRB = allOutputs;
  PORTB = 0x00;
  delay(200);


}

void loop() {

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
        }else if(counter < 70 && counter > 50){
            Serial.println("Solid");
          }else{
             Serial.println("Not Sure");
            }

     delay(50);
    PORTB ^= 0x08; //Down
    //delay(24)
  
    
     timerB = micros();
     delay(200);
     
     Serial.print("Counter: ");
     Serial.println(counter);
     //Serial.println(timerB - timerA);   
//}

  // put your main code here, to run repeatedly:

}
