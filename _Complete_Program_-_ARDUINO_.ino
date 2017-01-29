/* COMPLETE PROGRAM - Motor Control and RPM Indicator */

#include <Servo.h>
Servo m1; // Identifying ESC/Motor
int microseconds;
int b; //number of blades
float throttle=0.0;
float rpm;
int remainingByte = 0;
typedef unsigned char uchar;
uchar signal = 4; //RPM Sensor on pin 4
uchar rpsout = 11; //Data out to labjack
uchar throut = 3; //Data out to labjack

void setup() {
  Serial.begin(9600); // Opening communication
  Serial.println("COMPLETE PROGRAM - Motor Control and RPM Indicator");
  Serial.println("Warning: Do not connect power supply.");
  pinMode(signal, INPUT); //RPM Sensor pin
  pinMode(rpsout, OUTPUT);   // sets the pin as output
  pinMode(throut, OUTPUT);  
  
  /* Initial Motor Setup */
  m1.attach(9); // ESC/Motor attatched to pin 9
  throttle = 0.0;
  microseconds = int((mapfloat(throttle, 0.0, 100.0, 1092.0, 1904.0))+0.50); //round to int
  m1.writeMicroseconds(microseconds);

  delay(1000);
  Serial.println("\nPlease turn on the power supply.");

  while (Serial.available() == 0); // Waiting for 'blank' input
  while (Serial.available() != 0) {
    byte ch = Serial.read();
  }

  Serial.println("Waiting to initialize...");
  Serial.println(" ");
  delay(1000);


  /* Number of Blades, b */
  Serial.println("Please enter the number of blades on the propeller 'b', Press ENTER:");
  while (Serial.available() == 0) ;  // Wait until integer input
  {
    b = Serial.parseInt();
    Serial.print("b = ");
    Serial.println(b);
  }
  delay(1000);
  Serial.println("\nReady to Stream.");
  Serial.println("Enter a throttle percentage value at anytime.");
  Serial.println(" ");
  Serial.println("Time(ms)  Throttle(%)  rev/s  RPM");
  delay(2000);
  //!END OF SETUP!
}



void loop() {

  /* Motor Control */
  Serial.print(millis());
  Serial.print("  ");
  Serial.print(throttle); // Only read with a serial input given
  Serial.print("  ");
  analogWrite(throut, int(throttle*2.54)+0.5); //send throttle data to labjack 
  
  if (Serial.available() > 0) {
     throttle = Serial.parseFloat(); //read float
     while(Serial.available() > 0) {        
        remainingByte = (Serial.read()); //clear remaining values 
      }
    //Error Test
    if (throttle < 0.0 || throttle > 100.0) {
      throttle = 0.0;
      microseconds = int((mapfloat(throttle, 0.0, 100.0, 1092.0, 1904.0))+0.50); //round to int
      m1.writeMicroseconds(microseconds); // Motor will power down to 0%
    }
    microseconds = int((mapfloat(throttle, 0.0, 100.0, 1092.0, 1904.0))+0.50); //round to int
    m1.writeMicroseconds(microseconds); // Will write to new throttle
    delay(100);
  }
  if (throttle > 9) {
    rpm = getrpm(b, rpm); //pass the last rpm
  }
  else {
    rpm = 0.00;
    
    Serial.print(0.00); //rps
    Serial.print("  ");
    
    analogWrite(rpsout, 0); //send pwm data to labjack 
  }
  Serial.print("  ");
  Serial.println(rpm);
}

float getrpm( int b, float rpmlast ) {

  unsigned long time1 = 0;
  unsigned long time2 = 0;
  unsigned long elapsedtime = 0;
  float rps = 0;
  //float rpm = 0;
  float rpmnew = 0;
  //float rpmlast = 0;
  int count = 0;
  int propState = 0;
  int lastpropState = 0;



  /* RPM Indicator */
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
  Serial.print("  ");
 
  analogWrite(rpsout, int((rps*1.8)+0.5)); //send pwm data to labjack 
  
  rpmnew = rps * 60;

  //Serial.print(rpmnew);
  //Serial.print(" ");

  //smoothing equation rpm = (rpmlast*alpha) + (rpmnew *(1-alpha))
 
  rpm = (rpmlast * 0.75) + (rpmnew * 0.25);

 
  
  //Serial.println(rpm);

  //elapsedtime = 0;
  //time1 = 0;
  //time2 = 0;

  //return rpmnew;
  return rpm;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
