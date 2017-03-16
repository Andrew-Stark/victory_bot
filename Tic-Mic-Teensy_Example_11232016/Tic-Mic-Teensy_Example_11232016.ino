#define ARM_MATH_CM4
#include <arm_math.h>

/******************************
 *  Teensy Pin Assignment
 * ***************************/
//If using the example loop I made, have these in successive # order (i.e. TIC1=14,TIC2=15,MIC1=16,MIC2=17)
#define TIC1_TRACER_PIN  14         //Plug the output of your Tic Tracer into this I/O pin on the Teensy.
#define TIC2_TRACER_PIN  15         //Plug the output of your Tic Tracer into this I/O pin on the Teensy.
#define MIC1_AUDIO_PIN   16         //Plug the output of your Electret Microphone into this I/O pin on the Teensy.
#define MIC2_AUDIO_PIN   17         //Plug the output of your Electret Microphone into this I/O pin on the Teensy
#define MOSFET_PIN 18
#define ARDUINO_PIN 19

/*************************************
 * Analog Input Signal Read Settings
 * ***********************************/
#define ANALOG_READ_RESOLUTION 10 //This is the ADC reading resolution. I.E.: 10-bit=0-1024 (UNO/MEGA maximum), 13-bit=0-8192 (Teensy Usable Maximum), 16-bit=0-65536 (Teensy Actual Maximum Using Precision AREF Resistor)
#define ANALOG_READ_AVERAGING  100 //If you want to average X number of readings together PER analog read, before putting that number into the array that will be used to perform the FFT, this is where you adjust raw analog sample reading averaging.


/************************
 *  FFT Variables
************************/
int SAMPLE_RATE_HZ = 1000;     // (Possible Teensy Range: 100-9000Hz)
                               //Lower frequency = less frequencies per bin.
const int FFT_SIZE = 256;      //Number of bins

IntervalTimer samplingTimer;        //Interval Timer Type (up to 4): uses interrupts to call a function at a precise time. 
                                    //Initializing function: IntervalTimer.begin(function-to-run,microseconds-between-runs).
                                    
float samples[FFT_SIZE*2];          //FFT array. Sample count should be 2x the samples you're trying to discretize.
float magnitudes[FFT_SIZE];         //This is the array of magnitude values output by the FFT algorithm.
                                    //These magnitudes are the magnitudes in each 'bin'. If 60Hz falls in Bin 15, you should call on magnitudes[15]
                                    //yourbinmagnitudevalue = 20.0*log10(magnitudes[##]) can convert it to decibels if you want. 
                                    
int sampleCounter = 0;              //sampleCounter global variable says how many analog samples have been logged to the samples array, resets each time sampling completes.
int sampleSource = TIC1_TRACER_PIN; //This tells the sampleBegin() function which source to sample from. By default I set it to the TIC1_INPUT_PIN (Digital 14)

boolean wireFlag = false;

void setup() {
  Serial.begin(38400); //Set up serial port and set communication baud rate to 38400
  pinMode(TIC1_TRACER_PIN, INPUT); //initialize the I/O pin as input
  pinMode(TIC2_TRACER_PIN, INPUT); //initialize the I/O pin as input
  pinMode(MIC1_AUDIO_PIN, INPUT); //initialize the I/O pin as input
  pinMode(MIC2_AUDIO_PIN, INPUT); //initialize the I/O pin as input
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(ARDUINO_PIN, OUTPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION); //Tells the Teensy to perform analog reads at the resolution you set above.
  analogReadAveraging(ANALOG_READ_AVERAGING);   //Tells the Teensy to average X number of samples per each sample.
  samplingBegin(); //First things first, gather samples before we even get into the loop.

  delay(1000);
  powerSwitchTracer();
 // Serial.println("millis,,Source,,Magnitude,,");//Header just for CSV Excel Import 
}

void loop() {
  if(samplingIsDone()){
    ////Calculate the FFT if a fresh sample set is available.///////// Reference the CMSIS DSP Library:
    //////////////////// Run FFT on sample data.////////////////////// https://www.keil.com/pack/doc/CMSIS/DSP/html/structarm__cfft__radix4__instance__f32.html
    arm_cfft_radix4_instance_f32 fft_inst;                        //Calls the ARM FFT Library, sets up fft_inst instance for a CFFT of radix4 (base4) at single precision floating point 32-bit numbers.
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);          //Tells the FFT Library you're using instance 'fft_inst', the FFT Bin Size is 'FFT_SIZE', ifftFlag '0' tells it to perform a forward direction FFT (as opposed to inverse '1'), and bitReverseFlag '1' says we want the output bits to stream in the reverse direction.
    arm_cfft_radix4_f32(&fft_inst, samples);                      //Control structure initialization for FFT function
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);             // Calculate magnitude of complex numbers output by the FFT. 'samples' is interleaved as [real, imag, real, imag, real, imag,...] and the first bin is the total bin magnitude (basically ignore it).
    //////////// END FFT Code: Output is magnitudes[] array //////////
    

    ///////////EXAMPLE OUTPUT////////////////
    Serial.print("millis:,");              //Print the number of milliseconds the code has been running
    Serial.print(millis());                //
    Serial.print(",Source:,");             //Which analog input source was that?
    Serial.print(sampleSource);            //
    int i;                                 //Declare FOR loop count variable
    for (i=15; i < 20; i = i + 1){         //FOR loop i=## is lower bin to print,  i < ## is upper bin to print, plus 1.
      int magOut = (int)(magnitudes[i]);   //Declare freq as the value in magnitude array value [i]
      Serial.print(",");                   //Tell me which bin that magnitude was from
      Serial.print(magOut);                //
                                           //(The overuse of commas makes it easier to Excel import as CSV)
    }                                      //
   
    if(magnitudes[15] < 500){
      wireFlag = false; 
      Serial.print("NOPE"); 
      digitalWrite(ARDUINO_PIN, LOW);   
    }
    else{
      wireFlag = true;
      Serial.print("WIRE!!!!");
      digitalWrite(ARDUINO_PIN, HIGH);
    }
     Serial.println(",");                 //
    /////////////////////////////////////////

      //Set the sampling source
      if(sampleSource == TIC1_TRACER_PIN){
        //Restart audio sampling.
        samplingBegin();
       // sampleSource = sampleSource + 2; //change to +1 if using 2nd tic tracer
      }
//      else if(sampleSource == TIC2_TRACER_PIN){
//        samplingBegin();                          //uncomment if using 2nd tic tracer
//        sampleSource = sampleSource + 1;
//      }
      else if(sampleSource == MIC1_AUDIO_PIN){
        samplingBegin();
//        sampleSource = sampleSource + 1;
//      }
//      else if(sampleSource == MIC2_AUDIO_PIN){
//        samplingBegin();                           //uncomment if using 2nd microphone
        sampleSource = TIC1_TRACER_PIN;
      }
      else Serial.println("Invalid Source Configuration");
      delay(300);

  }
      //////END samplingIsDone() FFT functions////
    
}
///////////END LOOP////////////


////////////////////////////////////////////////////////////////////////////////////////
//////////SAMPLING FUNCTIONS////////This the the function for taking analog samples.////
////////////////////////////////////these samples are fed to the FFT when full./////////
void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(sampleSource);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
}
//////////END SAMPLING FUNCTIONS////////////

/************************************************************/

/*********************************
 *   Arduino Interfacing
 *********************************/
void powerSwitchTracer()
{
  digitalWrite(MOSFET_PIN, HIGH);
  delay(2000);
  digitalWrite(MOSFET_PIN, LOW);    
}

