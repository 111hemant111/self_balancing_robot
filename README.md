# PID-controlled Self-balancing Robot
This project implements an inverted pendulum concept to build a self-balancing robot. An Inertial Measurement Unit is used to detect the angle of inclination and these values are continuously fed to a PID Controller which would then provide the amount of torque that's needed to be applied in the direction of fall to keep the robot balanced.

The body and chassis of this robot was designed on Autodesk Fusion 360 and built using Lulzbot Mini 3D Printer. Users can replicate similar behaviour by using any other material.

## Status
Completed. Last update Dec 2017. Refer to 'Scope for Improvement' section.

## Components used
1. Arduino Mega 2560
2. MPU-6050 Inertial Measurement Unit
3. US-020 Ultrasonic sensor
4. L298N Motor Driver
5. 9V Batteries - 3
6. 16x2 LCD Display
7. Trimming Potentiometer
8. Misc. tools

## Finished product
![Final product](docs/self_balancing_robot.JPG?raw=true "Final product")

## Scope for Improvement
This is a fully functional final version. Following improvements can be made:
1. Smoother balancing.
2. Ability to steer.
Please refer to [schematics](/schematics) and [docs](/docs) to improve upon the progress.

## License
&copy; 2017 Hemant Kuruva

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so.
