#include <AccelStepper.h>
#define POT_PIN A0
#define POT_MIN_VAL -1
#define POT_MAX_VAL 1
#define SAMPLE_TIME 10 // Sample potentiometer four times a second

AccelStepper stepper(AccelStepper::DRIVER, 6, 5);

unsigned long prevTime = 0;
double kp = .025, ki = .05, kd = .001;
double prev_error = 0, integral = 0, derivative = 0, setpoint = 0;
unsigned long prev_time = 0, previous_Time = 0;
double input = 0, input2, current_position = 0, prev_position = 0, output = 0;
String serial_input;


void setup() {
  pinMode(POT_PIN, INPUT);
  Serial.begin(9600);
}


void loop() {
  //delay(1);

  if(Serial.available() > 0){
    Serial.print(Serial.available());
    serial_input = Serial.read();
    serial_input = Serial.readStringUntil(',');
    input = 5 * atof(serial_input.c_str());
    
  }
  
  unsigned long current_sample_Time = millis() + 1;
  unsigned long current_Time = millis() + 1;

  if (current_sample_Time - prevTime >= SAMPLE_TIME) {
    prevTime = current_sample_Time;
    int potVal = analogRead(POT_PIN);
    input2 = potVal;

    
    float potPercent = (float)potVal / (float)POT_MAX_VAL;
    
    int servoAngle = map(input, POT_MIN_VAL, POT_MAX_VAL, 0, 360);
    setpoint = 1 * servoAngle;//1.11;
  }

  if (current_Time - previous_Time >= 50) {

    double error = setpoint - current_position;

    Serial.print(" setpoint: ");
    Serial.print(setpoint/1.11);

    integral += error;// * (current_Time - previous_Time) / 1000.0;
    derivative = (current_position - prev_position) / ((current_Time - previous_Time) / 1000.0);
    
    // Update previous values for next iteration
    prev_position = current_position;
    prev_error = error;
    previous_Time = current_Time;

    current_position = kp * error + ki * integral + kd * derivative;

    Serial.print(" pos: ");
    Serial.println(current_position/1.11);
    
  }


//---------------------------
   
   //analogWrite(6, 1);
   //delay(1000);
   //analogWrite(6, -1);
   //delay(1000);
    stepper.moveTo(current_position);
    stepper.setMaxSpeed(600000000);
    //stepper.setSpeed(5000);
    stepper.setAcceleration(10000);
    stepper.run();

}
