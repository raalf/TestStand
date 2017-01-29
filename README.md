Arduino code for RAALF's propeller test stand.

Digital Pins:
- RPM Sensor on pin 4
- ESC out on pin 9
- RPS out on pin 11

Notes:
- RPS sent out as a pwm pulse (8bit duty cycle, 0-255) according to equation RPS = pwm*1.8. Maximum 8500RPM.
