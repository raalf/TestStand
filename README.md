Arduino code for RAALF's propeller test stand.

Digital Pins:
- RPM Sensor on pin 4
- ESC out on pin 9 (timer 1)
- RPS out on pin 11 (timer 2)
- Throttle out on pin 3 (timer 2)

Notes:
- RPS sent out as a pwm pulse (8bit duty cycle, 0-255) according to equation RPS = pwm/1.8. Maximum 8500RPM. Resolution of 33.33RPM.
- Throttle % sent out as a pwm pulse (8 bit duty cycle, 0-255) according to equation Throttle = pwm / 2.54. Resoltuion of 0.39 Throttle %. 
- millis() / micros() runs on timer 0
- must run ljstream at at least 5000hz to recieve rps and throttle data. 
