###########################################################
      Team 3167 RaspberryPi Logo Tracker Application
###########################################################

TODO:
- Figure out way to launch application on startup, so as soon
  as robot power is applied, we're ready to rock and roll.
- Consider different camera communication methods:
	- Use Axis Cameras that we already have
	- Use cheap USB cameras connected directly to RPi unit
		- This would allow us to possibly to stereo vision!
		- This also makes the camera(s) used for logo detection
		  independent of camera feed to driver station - we can
		  put these cameras wherever is most useful for logo
		  detection (possibly very low) but mount the DS camera
		  where it is most useful for drivers (possibly very high)