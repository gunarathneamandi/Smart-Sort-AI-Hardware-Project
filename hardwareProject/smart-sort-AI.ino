#define EIDSP_QUANTIZE_FILTERBANK   0

#include <PDM.h>
#include <AIsortbin_inferencing.h>
#include <AccelStepper.h>
#include <Servo.h>

/** Audio buffers, pointers and selectors */
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/**
 * @brief      Arduino setup function
 */

AccelStepper stepper1(1, 2, 3); // (Typeof driver: with 2 pins, STEP, DIR)

int compartment[] = {1220, 635,  50, 0, -580};//(Ball, Bottle, Can ,Noise, Paper)
const int HALL = 5;


Servo trapdoor;
int pos = 0;  
float threshold = 0.5;
int selected = 3; //default as Noise


//Manual setup buttons and switch
const int switchPin = 12; // Pin where the switch is connected
const int button1 = 7;
const int button2 = 6;
const int button3 = 10;
const int button4 = 11;


bool isHoming=false;

void homeStepperMotor() {
    isHoming=true;
    // Home the stepper motor
    stepper1.setMaxSpeed(2000); // Set maximum speed value for the stepper
    stepper1.setAcceleration(1000); // Set acceleration value for the stepper

    while (digitalRead(HALL)) {
        stepper1.setSpeed(-800); // Move motor in the negative direction
        stepper1.runSpeed(); // Run the motor at the set speed  
        Serial.println("homing...");
    }
    delay(100); // Delay to let the motor stabilize
    stepper1.setCurrentPosition(0); // Set the current position to 0
    isHoming=false;
}

void setup() {
  pinMode(switchPin, INPUT_PULLUP);// Set the switch pin as input with internal pull-up resistor
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  
  Serial.begin(115200);// Start the serial communication for debugging
  pinMode(HALL, INPUT);
  trapdoor.attach(9);
  trapdoor.write(0); 

  
   
  homeStepperMotor();
  //end of stepper motor calibration


    // servo motor calibration
  for (pos = 0; pos <= 80; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    trapdoor.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 80; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    trapdoor.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15 ms for the servo to reach the position
  }
    // end of servo motor calibration
    delay(500); //Delay so the sound of servo is not captured

    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    //while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }


}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */

void loop() {
  int switchState = digitalRead(switchPin); // Read the state of the switch
  int button1state = digitalRead(button1);
  int button2state = digitalRead(button2);
  int button3state = digitalRead(button3);
  int button4state = digitalRead(button4);

  if (switchState == HIGH) {
    // The switch is pressed (connected to ground)
    Serial.println("AI Model on");
    // ei_printf("Starting inferencing in 5 seconds...\n");
    // delay(5000);
    

    // ei_printf("Recording in 1...\n");
    delay(300);

    while (true) {
      // if (isHoming) {
      //   // Skip inference if homing is in progress
      //   continue;
      // }
      
      if(!(digitalRead(switchPin))) break;

      threshold = 0.5;
      bool m = microphone_inference_record();
      if (!m) {
          ei_printf("ERR: Failed to record audio...\n");
          return;
      }

      //ei_printf("Recording done\n");

      signal_t signal;
      signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
      signal.get_data = &microphone_audio_signal_get_data;
      ei_impulse_result_t result = { 0 };

      EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
      if (r != EI_IMPULSE_OK) {
          ei_printf("ERR: Failed to run classifier (%d)\n", r);
          return;
      }

      // print the predictions
      //ei_printf("Predictions ");
      ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
          result.timing.dsp, result.timing.classification, result.timing.anomaly);
      ei_printf(": \n");
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
          //ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
          if (result.classification[ix].value > threshold){
            ei_printf("\nYOUR TRASH IS: %s\n", result.classification[ix].label);
            selected = ix;
            threshold=result.classification[ix].value;
          }
      }
      ei_printf("\nYOUR final TRASH IS: %d\n",selected);
    
      if (selected == 0){
        Serial.println("ball");     
        break;
      }
      else if (selected == 1){
        Serial.println("bottle");
        break;
      }
      else if (selected == 2){
        Serial.println("can");
        break;
      }
      else if (selected == 4){
        Serial.println("paper");
        break;
      }
      else {
        Serial.println("noise");
      }
    
    #if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
    #endif
    //delay(1000);
    }
    ////////////////////////////////////////
    
    if (selected == 0 || selected == 1 || selected == 2 || selected == 4){
     
      compartmentSelecting(selected);
      homeStepperMotor();

    }

  } else {
    // The switch is not pressed
    Serial.println("Manual model");
    if (button1state== LOW){
      Serial.println("Button 1 pressed = Ball ");
      selected = 0;
      delay(500);
      compartmentSelecting(selected);
      homeStepperMotor();
       }
    else if (button2state==LOW)
    {
      Serial.println("Button 2 pressed = Bottle ");
      selected = 1;
      delay(500);
      compartmentSelecting(selected);
      homeStepperMotor();

      }
    else if (button3state==LOW)
    {
      Serial.println("Button 3 pressed = Can ");
      selected = 2;
      delay(500);
      compartmentSelecting(selected);
      //homeStepperMotor();
      }
    else if (button4state==LOW)
    {
      Serial.println("Button 4 pressed = Paper ");
      selected = 4;
      delay(500);
      compartmentSelecting(selected);
      homeStepperMotor();
      }

      
  }


  delay(100); // Small delay to avoid bouncing issues
}


void compartmentSelecting(int selected){
  stepper1.moveTo(compartment[selected]);
      stepper1.runToPosition();
      delay(500);
      for (pos = 0; pos <= 80; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        trapdoor.write(pos);              // tell servo to go to position in variable 'pos'
        delay(5);                       // waits 15 ms for the servo to reach the position
      }

      delay(2000);

      for (pos = 80; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        trapdoor.write(pos);              // tell servo to go to position in variable 'pos'
        delay(5);                       // waits 15 ms for the servo to reach the position
      }
      delay(500);
}




/**
 * @brief      PDM buffer full callback
 *             Get data and call audio thread callback
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if (inference.buf_ready == 0) {
        for(int i = 0; i < bytesRead>>1; i++) {
            inference.buffer[inference.buf_count++] = sampleBuffer[i];

            if(inference.buf_count >= inference.n_samples) {
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffer == NULL) {
        return false;
    }

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    PDM.setBufferSize(4096);

    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start PDM!");
        microphone_inference_end();

        return false;
    }

    // set the gain, defaults to 20
    PDM.setGain(80);

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;

    while(inference.buf_ready == 0) {
        delay(10);
    }

    return true;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    free(inference.buffer);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif