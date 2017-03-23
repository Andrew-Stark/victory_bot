
#define arrayIndex 10000        //may have to either decrease or increase the number of sample data reading that we take



int Threshold = 550;          //if analogRead is greater than this value then increase counter

int getData[arrayIndex] = {0};      //this is the sample data

int counter = 0;              //counts of signal going above the threshold
int micro= 9;                 //microphone pin number

int thumperPin = 29;          //thumper pin number


void setup() {
  Serial.begin(9600);
  analogReadResolution(10);       //10 bit resolution
  // put your setup code here, to run once:
pinMode (thumperPin, OUTPUT);   
delay(200);                       //wait to get initialized as output

}

void loop() {
  
 // digitalWrite(thumperPin,
 
digitalWrite(thumperPin, HIGH);     //hit
delay(25);                          //wait for the thumper to be close to the ground
for (int i =0; i<arrayIndex;i++){
  getData[i] = analogRead(9);       //sample data
  if (getData[i]>Threshold)         //if the sampled data is greater than threshold increase counter
    counter++;
}
delay(500);

digitalWrite(thumperPin,LOW);     //take it up
delay(500);


Serial.print("Counter = ");
Serial.println(counter);          //Display counter. It is different for hollow and foam


//Average = 0;

counter =0;
    
  
  // put your main code here, to run repeatedly:

}
