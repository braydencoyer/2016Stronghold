Base code for 2016 FIRST Stronghold Team 4381.

Things included in this project:
- NavX-MXP used for angle turning
	 - Make sure you have the libraries installed in the default directory before cloning and opening in Eclipse! If you don't, your gonna have a bad time.
- Robot Drive on 4 PWN Victors w/ double solenoid shifter
- Full SmartDashboard setup
	-Stores each defense (for autobreach buttons if we ever need 'em)
	-Starting position selection
	-Auto mode selection (many just for testing)
		-Full Auto=Breach,shoot w/ vision
		-'' w/ return=Back up (under low bar ONLY)
		-Breach only
		-Reach only forward/reverse (see below)
		-Do nothing (auto off)
- Switcher to breach a given defense position
- Main joystick (arcade drive) and specials joystick
- Shooter w/ 2 Talons (on CAN) for shooting, solenoid for pushing ball, and angle motor
- Breach autonomous for all defenses except Sally Port/Drawbridge
- Feelers (solenoids in front)
- Corrected Drive: Uses NavX angle to drive perfectly straight
- Approach: Uses NavX to approach defense ramp, detects angle change to determine when we have made it
- RotateToAngle: rotate to an angle using NavX
- Functions to move encoder motors to specific values
- Top and bottom limit switches for angle motor
- Andy's Arm (single part, CAN Talon)
- Vision processing, automatic aiming & calibration mode
- Channels.h file for storing channel numbers (PWM, CAN, button numbers)
- LimitSwitch class: A class that allows changing of limit switches from/to normally open/closed quickly

Branches:
This section is to document all current branches and their status.
- master: Master branch, currently functional competition code. Small changes/tuning only please, unless a change in general robot design (if experimental, create a new branch).
- NavXStuff: Initial integration of the NavX-MXP device. Branch pulled into master at completion, now closed to further edits.
- LimitSwitchClass: Initial writing and experimenting with the LimitSwitch class. Class is complete and functional. Branch pulled into master at completion, now closed to further edits.
- USBPermaMode: Migration from IMAQ camera management to use of the USBCamera class. This was intended to allow permanent camera configuration to disable automatic brightness recalibration, which made reliable targeting impossible. Turns out the class is just an awful wrapper for IMAQ, and spews errors constantly. Branch not pulled into master, now abandoned.
- IMAQdxSetAttribute: Attempting to use IMAQ to manually change properties in runtime memory of cameras. Requires setting raw values over the USB connection. Work in progress.
