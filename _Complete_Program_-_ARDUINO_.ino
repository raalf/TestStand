/* COMPLETE PROGRAM - Motor Control and RPM Indicator */

#include <Servo.h>
#include <PID_v1.h>

Servo m1; // Identifying ESC/Motor
int microseconds;
int b; //number of blades
char inData[20]; // Allocate some space for the string
char inChar = -1; // Where to store the character read
byte index = 0; // Index into array, for imcoming serial command
float throttle = 0.0;
float prev_throttle = 0.0;
float rpm;
int remainingByte = 0;
typedef unsigned char uchar;
uchar signal = 4; //RPM Sensor on pin 4
uchar rpsout = 11; //Data out to labjack
uchar throut = 3; //Data out to labjack

//Define PID Variables we'll be connecting to
double Setpoint, Input, Output;
//Define the aggressive and conservative Tuning Parameters
//0.03
double aggKp = 0.004, aggKi = 0.02, aggKd = 0.01 ; //close to target
double consKp = 0.004, consKi = 0.01, consKd = 0.00; //far away from target
int PIDon = 0; //PID on off
int pidinit = 0; //PID init loop
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup() {
  Serial.begin(9600); // Opening communication
  myPID.SetMode(MANUAL);  //start with PID OFF
  Serial.println("\n+---------------+       +------------------------------------------------+");
  Serial.println("| RYERSON       |       | RPM and Motor Control                          |");
  Serial.println("| APPLIED       |       | https://github.com/raalf/TestStand             |");
  Serial.println("| AERODYNAMICS  |       | Throttle %: Enter ## (0 - 100)                 |");
  Serial.println("| LABORATORY OF |       | Hold Current RPM: Enter h                      |");
  Serial.println("| FLIGHT        |       | Hold Specific RPM: Enter h #### (1000 - 9000)  |");
  Serial.println("+---------------+       +------------------------------------------------+\n");


  Serial.println("Warning: Do not connect power supply.");
  pinMode(signal, INPUT); //RPM Sensor pin
  pinMode(rpsout, OUTPUT);   // sets the pin as output
  pinMode(throut, OUTPUT);

  /* Initial Motor Setup */
  m1.attach(9); // ESC/Motor attatched to pin 9
  throttle = 0.0;
  microseconds = int((mapfloat(throttle, 0.0, 100.0, 1092.0, 1904.0)) + 0.50); //round to int
  m1.writeMicroseconds(microseconds);

  delay(1000);
  Serial.println("\nPlease turn on the power supply.");

  while (Serial.available() == 0); // Waiting for 'blank' input
  clearbuffer();

  Serial.println("Waiting to initialize...");
  Serial.println(" ");
  delay(1000);

  /* Number of Blades, b */
  Serial.println("Please enter the number of blades on the propeller 'b', Press ENTER:");
  while (Serial.available() == 0) {
  }
  // Wait until input
  b = Serial.parseInt();
  clearbuffer(); //clear serial buffer
  Serial.print("b = ");
  Serial.println(b);

  delay(1000);
  Serial.println("\nReady to Stream.\n");
  Serial.println("Time(ms) Throttle(%) RPS   RPM   PID STATE (ERROR)");


  delay(2000);
  //!END OF SETUP!
}



void loop() {

  //////////////////////////// PRINT DATA /////////////////////////////
  Serial.print(millis()); //time since boot
  Serial.print("\t");
  Serial.print(throttle); //throttle position %
  Serial.print("\t");
  analogWrite(throut, int(throttle * 2.54) + 0.5); //send throttle data to labjack

  //////////////////////////// SERIAL INPUT EVENT /////////////////////
  if (Serial.available() > 0) {

    prev_throttle = throttle; // store throttle setting from prev. instance

    //    throttle = Serial.parseFloat(); //read float


    while (Serial.available() > 0) { // Don't read unless

      if (index < 19) // One less than the size of the array
      {
        inChar = Serial.read(); // Read a character
        if (inChar == '\n' || inChar == '\r') {
          clearbuffer();
          break;
        }
        inData[index] = inChar; // Store it
        index++; // Increment where to write next
        inData[index] = '\0'; // Null terminate the string
      }
    }
    clearbuffer();


    if (inData[0] == 'h') { //PID EVENT
      String withoutH = String(inData);
      withoutH.remove(0, 1);
      PIDon = 0; // PID is on only in specific cases
      // check if without H contains valid float
      if (withoutH.toFloat() != 0) { // withoutH IS valid float
        if (withoutH.toFloat() > 1000 && withoutH.toFloat() < 9000) { // check if entered rpm is > 1000 & < 9000
          Setpoint = int(withoutH.toFloat() + 0.5); // set the rpm
          PIDon = 1;
        }
        else { // entered rpm is out of range
          PIDon = 0;
        }
      }
      else { // withoutH is not valid float or the float is zero
        if (strcmp(inData, "h")  == 0) { // only h is entered
          Setpoint = int(rpm + 0.5); //setpoint is current rpm rounded;
          PIDon = 1;
        }
        else {// do nothing
          if (strcmp(inData, "h0")  == 0) { // special case, throttle zero
            throttle = 0;
            microseconds = int((mapfloat(throttle, 0.0, 100.0, 1092.0, 1904.0)) + 0.50); //round to int
            m1.writeMicroseconds(microseconds); // Motor will power down to 0%
          }
          PIDon = 0;
        }
      }
      if (PIDon == 1 && rpm > 0) {
        Output = throttle; //trying to get the pid to not start at 0% throttle
        myPID.SetMode(AUTOMATIC);  //turn the PID on




        pidinit = 0;
        while (1 > 0) { //PID LOOP

          Serial.print(millis());
          Serial.print("\t");

          Serial.print(throttle); // Only read with a serial input given
          Serial.print("\t");
          analogWrite(throut, int(throttle * 2.54) + 0.5); //send throttle data to labjack

          Input = getrpm(b, rpm); //pass the last rpm
          Serial.print(rpm);
          Serial.print("\t");

          double gap = abs(Setpoint - Input); //distance away from setpoint

          if (gap > 100)
          { //we're far to setpoint
            myPID.SetTunings(consKp, consKi, consKd);
            Serial.print("PID GOTO ") ;
            Serial.print(Setpoint) ;
          }
          else
          {
            //we're close from setpoint
            myPID.SetTunings(aggKp, aggKi, aggKd);
            Serial.print("PID HOLD ");
            Serial.print(Setpoint) ;

          }

          myPID.Compute();
          if (pidinit != 0) { //this will ignore the very first pid loop (to deal with output starting at 0)
            throttle = Output;
          }

          pidinit = 1;

          Serial.print("\t");
          Serial.print("(");
          Serial.print(Setpoint - rpm);
          Serial.println(")");
          //Error Test
          if (throttle < 0.0 || throttle > 100.0) {
            throttle = prev_throttle;
            microseconds = int((mapfloat(throttle, 0.0, 100.0, 1092.0, 1904.0)) + 0.50); //round to int
            m1.writeMicroseconds(microseconds); // Motor will power down to 0%
          }

          microseconds = int((mapfloat(throttle, 0.0, 100.0, 1092.0, 1904.0)) + 0.50); //round to int
          m1.writeMicroseconds(microseconds); // Will write to new throttle
          delay(100);


          if (Serial.available() > 0) { // SERIAL EVENT
            myPID.SetMode(MANUAL);  //turn the PID off
            break;
          }
        }
      }
    }

    else { //THROTTLE EVENT
      if (strcmp(inData, "0")  == 0) {
        throttle = 0;
      }
      else if (strcmp(inData, ".")  == 0) {
        throttle = prev_throttle + 1;
      }
      else if (strcmp(inData, ",")  == 0) {
        throttle = prev_throttle - 1;
      }
      else if (strcmp(inData, ">")  == 0) {
        throttle = prev_throttle + 0.1;
      }
      else if (strcmp(inData, "<")  == 0) {
        throttle = prev_throttle - 0.1;
      }
      else if (String(inData).toFloat() != 0) {
        throttle = String(inData).toFloat();
      }
      else {
        throttle = prev_throttle;
      }



      //Error Test
      if (throttle < 0.0 || throttle > 100.0) {
        throttle = prev_throttle;
        microseconds = int((mapfloat(throttle, 0.0, 100.0, 1092.0, 1904.0)) + 0.50); //round to int
        m1.writeMicroseconds(microseconds); // Motor will power down to 0%
      }

      microseconds = int((mapfloat(throttle, 0.0, 100.0, 1092.0, 1904.0)) + 0.50); //round to int
      m1.writeMicroseconds(microseconds); // Will write to new throttle
      delay(100);


    } // END THROTTLE EVENT

    //clear inData buffer
    for (int i = 0; i < 19; i++) {
      inData[i] = 0;
    }
    index = 0;



  } // END OF SERIAL EVENT


  //////////////////////////// READ RPM /////////////////////////////
  if (throttle > 9) {
    rpm = getrpm(b, rpm); //pass the last rpm
  }
  else {
    rpm = 0.00;

    Serial.print(0.00); //rps
    Serial.print("\t");

    analogWrite(rpsout, 0); //send pwm data to labjack
  }
  Serial.print(rpm);
  Serial.print("\t");
  Serial.println("PID OFF");
}


//////////////////////////// FUNCTIONS /////////////////////////////
float getrpm( int b, float rpmlast ) {

  unsigned long time1 = 0;
  unsigned long time2 = 0;
  unsigned long elapsedtime = 0;
  float rps = 0;
  //float rpm = 0;
  int num_read = 4; //must be greater than 3
  float rpmnew[num_read];
  //float rpmlast = 0;
  int count = 0;

  int propState = 0;
  int lastpropState = 0;

  for (int i = 0; i < num_read; i++) {


    //wait for a prop to pass

    while (digitalRead(signal) == LOW)
    {
      //do nothing
    }
    while (digitalRead(signal) == HIGH)
    {
      //do nothing
    }

    count = 0;

    //count up to number of blades
    while (count < b + 1)
    {

      //if state has chaged...
      propState = digitalRead(signal);
      if (propState != lastpropState)
      {
        //if a prop is in the way
        if (propState == LOW)
        {
          if (count == 0) //if this is the opening pass, start timing
          {
            time1 = micros();
          }
          count++; //increase the count
          delay(2); //small delay necessary to remove debouncing
          //delayMicroseconds(500);
        }
      }
      lastpropState = propState;
    }

    //after one revolution (360 degrees) take the time
    time2 = micros() - 2000; // (!NOTE!..we subtract out the delay which was added after the last count++)
    //delay(50); //another small delay
    elapsedtime = time2 - time1;//get elapsed time for 360 deg revolution


    //Serial.print(elapsedtime);
    //Serial.print(" ");

    //find revs per second
    rps = 1 / ((elapsedtime) / 1000000.00);

    Serial.print(rps);
    Serial.print("\t");

    analogWrite(rpsout, int((rps * 1.8) + 0.5)); //send pwm data to labjack

    rpmnew[num_read]  = rps * 60;

    //Serial.print(rpmnew);
    //Serial.print(" ");
  }
    //smoothing equation rpm = (rpmlast*alpha) + (rpmnew *(1-alpha))

    //rpm = (rpmlast * 0.75) + (rpmnew * 0.25);

    int maximum = getIndexOfMaxValue(rpmnew, num_read);

    int minimum = getIndexOfMinValue(rpmnew, num_read);

    rpm = (array_sum(rpmnew, num_read) - rpmnew[maximum] - rpmnew[minimum]) / (num_read - 2);
    //Serial.println(rpm);

    //elapsedtime = 0;
    //time1 = 0;
    //time2 = 0;

 

  //return rpmnew;
  return rpm;
}

int getIndexOfMaxValue(float* array, int size) {
  int maxIndex = 0;
  int max = array[maxIndex];
  for (int i = 1; i < size; i++) {
    if (max < array[i]) {
      max = array[i];
      maxIndex = i;
    }
  }
}

int getIndexOfMinValue(float* array, int size) {
  int minIndex = 0;
  int min = array[minIndex];
  for (int i = 1; i < size; i++) {
    if (min > array[i]) {
      min = array[i];
      minIndex = i;
    }
  }
}

float array_sum(float * array, int size) {
  float sum = 0;
  for (int i = 1; i < size; i++) {
    sum = sum + array[i];
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void clearbuffer() {
  while (Serial.available() > 0) {
    remainingByte = (Serial.read()); //clear remaining values
  }
  Serial.flush();
}
