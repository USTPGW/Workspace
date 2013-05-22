This is the overall readme file for the project.

The goal of this project is to have a working kalman filter code that is capable
of communicating with one analog sensor, a 9-axis SPI sensor, a digital serial
GPS sensor, send out PWM signals for two servo motors as well as a propulsion motor,
and communicate serially with a peripheral digital device all within a set, timed framework.
The code should also store values periodically in case of communication failure.

To find the main file, simple enter the ADC3_DMA folder and find main.c.

It is called this because this was the name of the example that began all the coding work,
and by the time I realized I should be using a different title, the amount of work it
would take to adjust the toolchain was too great to warrant immediate fixing now.

This will all be fixed and cleaned up if time permits at the end of the project.

Also, code containing peripheral communcation will be added shortly.