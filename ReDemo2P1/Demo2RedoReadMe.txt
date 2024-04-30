In this Demo, our group integrated code from the raspberry PI (python) and the Arduino (C++). The raspberry PI
connected to a camera that detects an Aruco marker and sends an angle and distance away from the robot. The Arduino
then uses this information to turn this desired angle and drives the distance away to stop within a foot of the marker
In the first part of the Demo, the arduino instructs the motors to rotate until a marker is detected. When the 
marker is detected, the robot stops and adjusts its angle based on input from the PI and drives to the marker. 
The second part of the demo reads in a distance from the camera, goes that distance, stops,
rotates 75 degrees and drives in a circle around the marker, stopping in the same spot it started (Demo2Part2)

All hardware specifications and code setup is in the individual codes which will allow you to duplicate these demos.
Good luck!
