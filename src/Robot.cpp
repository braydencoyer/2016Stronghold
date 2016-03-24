//Last Edit: 3/22/16

#include "WPILib.h"
#include "AHRS.h"
#include "Channels.h"
#include "LimitSwitch.cpp"
//#include <chrono>

class Robot: public IterativeRobot
{
public:

	/* ----------------------------------------------------------------------
	 * 							DECLARATIONS
	 * ----------------------------------------------------------------------
	 */

	LiveWindow *lw = LiveWindow::GetInstance();

	//std::chrono::time_point<std::chrono::system_clock> start,end;

	//----------------------------AUTO MODE CONSTANTS-------------------
	//Thsese strings represent possible SmartDashboard auto modes/states/etc.
	const std::string AUTO_MODE_OFF = "Off";
	const std::string AUTO_MODE_FULL = "Full";
	const std::string AUTO_MODE_BREACH = "Breach";
	const std::string AUTO_MODE_APPROACH = "Approach";
	const std::string AUTO_MODE_APPROACH_RV = "ApproachRev";
	const std::string AUTO_MODE_ROTATETEST = "RotateTest";
	const std::string AUTO_MODE_VISIONTEST = "VisionTest";
	const std::string AUTO_MODE_FIRETEST = "FireTest";
	const std::string AUTO_MODE_RAISETEST = "RaiseTest";
	const std::string AUTO_MODE_DOUBLELB = "DoubleLowBar";
	const std::string AUTO_MODE_STRAIGHTTEST = "DriveStraightTest";

	const std::string BREACH_POS_LB = "LB";
	const std::string BREACH_POS_0 = "0";
	const std::string BREACH_POS_1 = "1";
	const std::string BREACH_POS_2 = "2";
	const std::string BREACH_POS_3 = "3";

	const std::string AUTO_DEFTYPE_PORT = "Port";
	const std::string AUTO_DEFTYPE_CHEV = "Checv";
	const std::string AUTO_DEFTYPE_MOAT = "Moat";
	const std::string AUTO_DEFTYPE_RAMPARTS = "Ramp";
	const std::string AUTO_DEFTYPE_BRIDGE = "Bridge";
	const std::string AUTO_DEFTYPE_SALLY = "Sally";
	const std::string AUTO_DEFTYPE_RT = "Rough";
	const std::string AUTO_DEFTYPE_RW = "RockWall";


	//-------------------AUTO MODE-----------------
	//Auto Mode
	SendableChooser *autoMode;

	//What defense is in each position
	SendableChooser *def[4];

	//Position number to breach
	SendableChooser *toBreach;

	//Strings and ints for holding the above
	std::string autoSelected;
	std::string defense[4];
	int breachPos;

	int autoState;


	//---------------------------NAVX CONSTANTS--------------------
	double ANGLE_TOLERANCE = 2;   //Degrees, how far can we be + or -

	//--------------------------VISION CONSTANTS-------------------
	int TARGET_ORIGIN_X = 333;
	int TARGET_ORIGIN_Y = 307;
	int ORIGIN_X_TOL = 15;
	int ORIGIN_Y_TOL = 18;

	Range RING_HUE_RANGE = {101, 155};	//Default hue range for ring light, old=64
	Range RING_SAT_RANGE = {225, 255};	//Default saturation range for ring light, old=88
	Range RING_VAL_RANGE = {240, 255};	//Default value range for ring light old=230
	float AREA_MINIMUM = 0.1; //Area minimum for particle as a percentage of total image area
	double VIEW_ANGLE = 60; //View angle for camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480

	double CAMERA_BRIGHTNESS_AUTO = 150;//TODO
	double CAMERA_BRIGHTNESS_DRIVING = 200;
	unsigned int CAMERA_WHITEBALANCE = 8000;
	double CAMERA_EXPOSURE = 20;
	//BRIGHTNESS: MAX: 255 MIN: 30
	//WHITE BALANCE: MAX: 10000 MIN: 2800
	//EXPOSURE: MAX: 20000 MIN: 5

	//-------------------MISC CONSTANTS-----------------------
	double MAX_IN_SPEED = .7;

	double LOWSHOT_SPEED = 0.8;

	//-----------------------MOTORS-------------------------
	Victor left, right;
	RobotDrive drive;

	CANTalon shooterA;
	CANTalon shooterB;
	CANTalon angleMotor;
	//One rotation: 9124
	const double SHOOTER_ANGLE_HOLD_PERCENT = 0.1; //Voltage percent needed to hold shooter in place

	CANTalon armMain;

	CANTalon kicker;

	//----------------------SENSORS---------------------

	//NavX Communication
	AHRS *ahrs;


	//---------------------HUMAN INPUT-------------------

	//Driver
	Joystick mainStick;

	//Specials
	Joystick specials;


	//-----------------------VISION------------------------

	//Particle measurements as output by IMAQ
	struct ParticleReport {
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
	};

	//Images: Frame stores RGB from camera, binaryFrame stores filtered image
	Image *frame;
	Image *rearFrame;
	Image *binaryFrame;

	//Filter function arguments
	ParticleFilterCriteria2 criteria[1];
	ParticleFilterOptions2 filterOptions = {0,0,1,1};

	//IMAQ storage
	IMAQdxSession session;
	int imaqError;

	//Register display names for our usb cameras
	const char* ATTR_VIDEO_MODE = "AcquisitionAttributes::VideoMode";//Str
	const char* ATTR_WB_MODE = "CameraAttributes::WhiteBalance::Mode";//Str
	const char* ATTR_WB_VALUE = "CameraAttributes::WhiteBalance::Value";//I64
	const char* ATTR_EX_MODE = "CameraAttributes::Exposure::Mode";//Str
	const char* ATTR_EX_VALUE = "CameraAttributes::Exposure::Value";//I64
	const char* ATTR_BR_MODE = "CameraAttributes::Brightness::Mode";//Str
	const char* ATTR_BR_VALUE = "CameraAttributes::Brightness::Value";//I64

	IMAQdxSession rearSession;

	//Position of target on-screen, to be used later.
	double screenPosX;
	double screenPosY;

	const int CYCLES_PER_FRAME = 5;

	//Info for switching the active camera
	bool rearCamActive;
	bool swapButtonPressed;

	ParticleReport best;

	//--------------------------PDP----------------------------------
	PowerDistributionPanel pdp;


	//-------------------------PNEUMATICS---------------------------
	//Shifter
	DoubleSolenoid shifter;

	Compressor compressor;

	//--------------------------LIMIT SWITCHES--------------------------
	LimitSwitch angleBottom;
	LimitSwitch angleTop;

	LimitSwitch hasBall;

	//--------------------------OTHER-----------------------------
	int cycle;

	Encoder leftEnc,rightEnc;

	//Kicker tech
	const static int KICKER_TICS_PER_REV = 471;
	bool kickerMoving;


	Timer timer;

	/*--------------------------------------------------------------
	 *						Initialization
	 * -------------------------------------------------------------
	 */

	Robot():
		left(CH_LEFTDRIVE),
		right(CH_RIGHTDRIVE),
		drive(left,right),
		shooterA(CH_SHOOTERA),
		shooterB(CH_SHOOTERB),
		angleMotor(CH_ANGLEMOTOR),
		armMain(CH_ARMMAIN),
		kicker(CH_KICKER),
		mainStick(CH_DRIVESTICK),
		specials(CH_SPECIALSTICK),
		pdp(CH_PDP),
		shifter(CH_SHIFTER_PCM,CH_SHIFTER_FW,CH_SHIFTER_RV),
		compressor(CH_PCMA),
		angleBottom(CH_SHOOTER_ANGLE_BOTTOM,true),
		angleTop(CH_SHOOTER_ANGLE_TOP,true),
		hasBall(CH_HASBALL,true),
		leftEnc(CH_ENC_L_A,CH_ENC_L_B),
		rightEnc(CH_ENC_R_A,CH_ENC_R_B),
		timer()
	{

		//--------Motor inversion----------------
		shooterB.SetInverted(true);
		shooterA.SetInverted(false);
		shooterA.SetFeedbackDevice(shooterA.QuadEncoder);

		kicker.SetFeedbackDevice(kicker.QuadEncoder);

		kicker.ConfigNeutralMode(kicker.kNeutralMode_Brake);
		shooterA.ConfigNeutralMode(shooterA.kNeutralMode_Brake);
		shooterB.ConfigNeutralMode(shooterB.kNeutralMode_Brake);
		angleMotor.ConfigNeutralMode(angleMotor.kNeutralMode_Brake);

		drive.SetInvertedMotor(drive.kRearLeftMotor,false);
		drive.SetInvertedMotor(drive.kRearRightMotor,false);

		angleMotor.SetFeedbackDevice(angleMotor.QuadEncoder);
		angleMotor.SetInverted(false);
	}


	void RobotInit()
	{
		drive.SetSafetyEnabled(false);
		left.SetSafetyEnabled(false);
		right.SetSafetyEnabled(false);
		angleMotor.SetSafetyEnabled(false);
		kicker.SetSafetyEnabled(false);

		shooterA.SetSafetyEnabled(false);
		shooterB.SetSafetyEnabled(false);

		cycle=0;

		angleMotor.SetEncPosition(0);

		armMain.SetSafetyEnabled(false);

		//--------------NAVX-MXP-------------------
		//Try to instantiate navx. If it fails, catch error so code doesn't crash.
		try {
			/* Communicate w/navX MXP via the MXP SPI Bus.                                       */
			/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception ex) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}


		//------------AUTO SELECTORS----------------
		//Selector for mode
		autoMode = new SendableChooser();
		autoMode->AddDefault("Auto Off", (void*)&AUTO_MODE_OFF);
		autoMode->AddObject("Full Auto", (void*)&AUTO_MODE_FULL);
		autoMode->AddObject("Breach",(void*)&AUTO_MODE_BREACH);
		autoMode->AddObject("Reach forward",(void*)&AUTO_MODE_APPROACH);
		autoMode->AddObject("Reach reverse",(void*)&AUTO_MODE_APPROACH_RV);
		autoMode->AddObject("Full Auto and Return",(void*)&AUTO_MODE_DOUBLELB);
		autoMode->AddObject("[TEST]Rotate",(void*)&AUTO_MODE_ROTATETEST);
		autoMode->AddObject("[TEST]Vision",(void*)&AUTO_MODE_VISIONTEST);
		autoMode->AddObject("[TEST]Fire",(void*)&AUTO_MODE_FIRETEST);
		autoMode->AddObject("[TEST]Raise Shooter",(void*)&AUTO_MODE_RAISETEST);
		autoMode->AddObject("[TEST]Drive Straight",(void*)&AUTO_MODE_STRAIGHTTEST);

		SmartDashboard::PutData("Auto Modes", autoMode);

		//Selector for starting position (where we are aiming)
		toBreach = new SendableChooser();
		toBreach->AddObject("1 (Lowbar)",(void*)&BREACH_POS_LB);
		toBreach->AddDefault("2",(void*)&BREACH_POS_0);
		toBreach->AddObject("3",(void*)&BREACH_POS_1);
		toBreach->AddObject("4",(void*)&BREACH_POS_2);
		toBreach->AddObject("5",(void*)&BREACH_POS_3);

		SmartDashboard::PutData("Breach", toBreach);

		//Selectors for each defense slot on field
		for(int j=0;j<4;j++)
		{
			def[j] = new SendableChooser();
		}

		def[0]->AddDefault("Defense2",(void*)&AUTO_DEFTYPE_BRIDGE);
		def[1]->AddDefault("Defense3",(void*)&AUTO_DEFTYPE_BRIDGE);
		def[2]->AddDefault("Defense4",(void*)&AUTO_DEFTYPE_BRIDGE);
		def[3]->AddDefault("Defense5",(void*)&AUTO_DEFTYPE_BRIDGE);

		for(int j=0;j<4;j++)
		{
			//Add each option
			def[j]->AddObject("Portcullis", (void*)&AUTO_DEFTYPE_PORT);
			def[j]->AddObject("Cheval", (void*)&AUTO_DEFTYPE_CHEV);
			def[j]->AddObject("Moat", (void*)&AUTO_DEFTYPE_MOAT);
			def[j]->AddObject("Ramparts", (void*)&AUTO_DEFTYPE_RAMPARTS);
			def[j]->AddObject("Drawbridge", (void*)&AUTO_DEFTYPE_BRIDGE);
			def[j]->AddObject("Sally Door", (void*)&AUTO_DEFTYPE_SALLY);
			def[j]->AddObject("Rock Wall", (void*)&AUTO_DEFTYPE_RW);
			def[j]->AddObject("Rough Terrain", (void*)&AUTO_DEFTYPE_RT);
		}

		SmartDashboard::PutData("DefenseSelectTwo",def[0]);
		SmartDashboard::PutData("DefenseSelectThree",def[1]);
		SmartDashboard::PutData("DefenseSelectFour",def[2]);
		SmartDashboard::PutData("DefenseSelectFive",def[3]);

		//------------------VISION---------------------
		//Create space in memory for images
		binaryFrame = imaqCreateImage(IMAQ_IMAGE_U8,0);
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		rearFrame = imaqCreateImage(IMAQ_IMAGE_RGB,0);

		//Opens the camera for communication. Returns a number other than IMAQdxErrorSuccess if something goes wrong.
		imaqError = IMAQdxOpenCamera("cam1", IMAQdxCameraControlModeController, &session);
		if(imaqError != IMAQdxErrorSuccess) {
			//Warn drivers that camera is dead
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}
		imaqError = IMAQdxConfigureGrab(session);

		if(imaqError != IMAQdxErrorSuccess) {
			//Tell drivers that we can't load the camera
			DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
		}

		//Open cam1
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &rearSession);
		if(imaqError != IMAQdxErrorSuccess) {
			//Warn drivers that camera is dead
			DriverStation::ReportError("IMAQdxOpenCamera 2 error: " + std::to_string((long)imaqError) + "\n");
		}
		//Configure the Grab session. Returns a number other than IMAQdxErrorSuccess if something bad happens.
		//imaqError = IMAQdxConfigureGrab(rearSession);
		if(imaqError != IMAQdxErrorSuccess) {
			//Warn drivers that camera is dead
			DriverStation::ReportError("IMAQdxConfigureGrab 2 error: " + std::to_string((long)imaqError) + "\n");
		}

		//Starts the session we configured above
		IMAQdxStartAcquisition(session);
		//IMAQdxStartAcquisition(rearSession);
		if(imaqError != IMAQdxErrorSuccess) {
			//Warn drivers that camera is dead
			DriverStation::ReportError("Configure session error: " + std::to_string((long)imaqError) + "\n");
		}

		rearCamActive = false;
		swapButtonPressed = false;

		//Put spaces for these numbers on dashboard. Used to adjust vision masking.
		SmartDashboard::PutNumber("Tote hue min", RING_HUE_RANGE.minValue);
		SmartDashboard::PutNumber("Tote hue max", RING_HUE_RANGE.maxValue);
		SmartDashboard::PutNumber("Tote sat min", RING_SAT_RANGE.minValue);
		SmartDashboard::PutNumber("Tote sat max", RING_SAT_RANGE.maxValue);
		SmartDashboard::PutNumber("Tote val min", RING_VAL_RANGE.minValue);
		SmartDashboard::PutNumber("Tote val max", RING_VAL_RANGE.maxValue);
		SmartDashboard::PutNumber("X To Target", TARGET_ORIGIN_X);
		SmartDashboard::PutNumber("Y To Target", TARGET_ORIGIN_Y);
		SmartDashboard::PutNumber("X Tol", ORIGIN_X_TOL);
		SmartDashboard::PutNumber("Y Tol", ORIGIN_Y_TOL);

		SmartDashboard::PutNumber("BrightnessTarget",CAMERA_BRIGHTNESS_AUTO);
		SmartDashboard::PutNumber("WhiteBalanceTarget",CAMERA_WHITEBALANCE);
		SmartDashboard::PutNumber("ExposureTarget",CAMERA_EXPOSURE);

		SetUpCamera();

		//-------------Encoders--------------
		//Reset all encoders if Talon power has not cycled
		angleMotor.SetEncPosition(0);
		leftEnc.Reset();
		rightEnc.Reset();
		armMain.SetEncPosition(0);
		kicker.SetEncPosition(0);

		kickerMoving = false;
	}
	/* ------------------------------------------------------------------------
	 * 									Teleop
	 * ------------------------------------------------------------------------
	 */

	//Called when Teleop starts
	void TeleopInit()
	{
		IMAQdxSetAttribute(session,ATTR_BR_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,CAMERA_BRIGHTNESS_DRIVING);
	}

	//Called repeatedly while Teleop is active
	void TeleopPeriodic()
	{
		if(!DriverStation::GetInstance().IsFMSAttached() && cycle==3)
		{
			SmartDashboard::PutNumber("Pitch",ahrs->GetPitch());
			SmartDashboard::PutNumber("Yaw",ahrs->GetYaw());
			SmartDashboard::PutNumber("Roll",ahrs->GetRoll());

			SmartDashboard::PutNumber("Left Encoder",leftEnc.Get());
			SmartDashboard::PutNumber("Right Encoder",rightEnc.Get());

			SmartDashboard::PutNumber("Angle Encoder",angleMotor.GetEncPosition());

			SmartDashboard::PutNumber("Arm Encoder",armMain.GetEncPosition());

			SmartDashboard::PutNumber("Shooter Encoder",shooterB.GetEncVel());

			SmartDashboard::PutNumber("Kicker Encoder",kicker.GetEncPosition());

			SmartDashboard::PutBoolean("Upper limit",angleTop.Get());
			SmartDashboard::PutBoolean("Lower Limit",angleBottom.Get());

			SmartDashboard::PutBoolean("Firing",kickerMoving);
		}

		SmartDashboard::PutNumber("Shooter Revs",shooterA.GetEncVel());

		SmartDashboard::PutBoolean("Has Ball",hasBall.Get());
		//---------------AUTOAIM-------------------
		//If button is pressed, use vision to line up
		if(specials.GetRawButton(BUT_AUTOAIMA))
		{
			IMAQdxSetAttribute(session,ATTR_BR_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,CAMERA_BRIGHTNESS_AUTO);
			AutoAimHardLoop();
			//Return halts all execution here
			IMAQdxSetAttribute(session,ATTR_BR_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,CAMERA_BRIGHTNESS_DRIVING);
			return;
		}
		//For vision calibration only, does not move robot, sends masked image to dash too
		if(specials.GetRawButton(BUT_AUTOAIMB))
		{
			IMAQdxSetAttribute(session,ATTR_BR_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,CAMERA_BRIGHTNESS_AUTO);
			CalibrateVision();
			return;
		}

		//---------------RESET BUTTONS------------------
		if(mainStick.GetRawButton(BUT_NAVX_RESET))
		{
			ahrs->ResetDisplacement();
			ahrs->ZeroYaw();
		}
		if(mainStick.GetRawButton(BUT_ENCODERS_RESET))
		{
			angleMotor.SetEncPosition(0);
			leftEnc.Reset();
			rightEnc.Reset();
			armMain.SetEncPosition(0);
		}

		//------------------AUTOBREACH----------------------
		//If button pressed, approach
		if(mainStick.GetRawButton(BUT_APPROACH))
		{
			CorrectedApproach(1,0);
			return;
		}
		//If button pressed, breach defType at selected position
		if(mainStick.GetRawButton(BUT_BREACH2))
		{
			Breach(0);
			return;
		}
		if(mainStick.GetRawButton(BUT_BREACH3))
		{
			Breach(1);
			return;
		}
		if(mainStick.GetRawButton(BUT_BREACH4))
		{
			Breach(2);
			return;
		}
		if(mainStick.GetRawButton(BUT_BREACH5))
		{
			Breach(3);
			return;
		}


		//------------------CAMERA FEED---------------------

		//Camera swappers
		if(mainStick.GetRawButton(BUT_SWAPCAMS)&&!swapButtonPressed)
		{
			//Save that button is pressed for next cycle
			swapButtonPressed=true;

			//Swap active camera
			rearCamActive=!rearCamActive;

			if(rearCamActive)
			{
				IMAQdxStopAcquisition(session);
				IMAQdxUnconfigureAcquisition(session);
				IMAQdxStartAcquisition(rearSession);
				IMAQdxConfigureGrab(rearSession);
			}
			else
			{
				IMAQdxStopAcquisition(rearSession);
				IMAQdxUnconfigureAcquisition(rearSession);
				IMAQdxCloseCamera(session);
				imaqError = IMAQdxOpenCamera("cam1", IMAQdxCameraControlModeController, &session);
				IMAQdxStartAcquisition(session);
				IMAQdxConfigureGrab(session);
				SetUpCamera();
				IMAQdxSetAttribute(session,ATTR_BR_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,CAMERA_BRIGHTNESS_DRIVING);
			}
		}
		else
		{
			//Save that button has been released
			if(!mainStick.GetRawButton(BUT_SWAPCAMS)) swapButtonPressed=false;
		}


		cycle++;
		//We don't want to send image every loop, send only every CYCLES_PER_FRAME cycles
		if(cycle>CYCLES_PER_FRAME)
		{
			cycle = 0;

			if(rearCamActive)
			{
				//Get rear camera
				IMAQdxGrab(rearSession,frame,false,NULL);
				CameraServer::GetInstance()->SetImage(frame);
			}
			else
			{
				//Get front camera
				//Get image
				IMAQdxGrab(session, frame, false, NULL);

				//Draw a target on the frame
				imaqDrawLineOnImage(frame,frame,DrawMode::IMAQ_DRAW_INVERT,{TARGET_ORIGIN_X-ORIGIN_X_TOL,TARGET_ORIGIN_Y-ORIGIN_Y_TOL},{TARGET_ORIGIN_X-ORIGIN_X_TOL,TARGET_ORIGIN_Y+ORIGIN_Y_TOL},0.0f);
				imaqDrawLineOnImage(frame,frame,DrawMode::IMAQ_DRAW_INVERT,{TARGET_ORIGIN_X-ORIGIN_X_TOL,TARGET_ORIGIN_Y-ORIGIN_Y_TOL},{TARGET_ORIGIN_X+ORIGIN_X_TOL,TARGET_ORIGIN_Y-ORIGIN_Y_TOL},0.0f);
				imaqDrawLineOnImage(frame,frame,DrawMode::IMAQ_DRAW_INVERT,{TARGET_ORIGIN_X+ORIGIN_X_TOL,TARGET_ORIGIN_Y-ORIGIN_Y_TOL},{TARGET_ORIGIN_X+ORIGIN_X_TOL,TARGET_ORIGIN_Y+ORIGIN_Y_TOL},0.0f);
				imaqDrawLineOnImage(frame,frame,DrawMode::IMAQ_DRAW_INVERT,{TARGET_ORIGIN_X-ORIGIN_X_TOL,TARGET_ORIGIN_Y+ORIGIN_Y_TOL},{TARGET_ORIGIN_X+ORIGIN_X_TOL,TARGET_ORIGIN_Y+ORIGIN_Y_TOL},0.0f);
				imaqDrawLineOnImage(frame,frame,DrawMode::IMAQ_DRAW_INVERT,{TARGET_ORIGIN_X,TARGET_ORIGIN_Y-70},{TARGET_ORIGIN_X,TARGET_ORIGIN_Y+70},0.0f);
				imaqDrawLineOnImage(frame,frame,DrawMode::IMAQ_DRAW_INVERT,{TARGET_ORIGIN_X-70,TARGET_ORIGIN_Y},{TARGET_ORIGIN_X+70,TARGET_ORIGIN_Y},0.0f);

				//Set image on camera server to frame
				CameraServer::GetInstance()->SetImage(frame);
			}
		}


		//-----------------DRIVE SYSTEM---------------------
		//Drive with main joystick values
		drive.ArcadeDrive(-mainStick.GetY(),mainStick.GetX());

		//-----------------SHIFTER--------------------------
		//Shift if button is pressed
		if(mainStick.GetRawButton(BUT_SHIFTER))
		{
			shifter.Set(shifter.kForward);
		}
		else
		{
			shifter.Set(shifter.kReverse);
		}

		//-------------------SHOOTER------------------

		//Speed based on throttle lever, convert from -1<t<1 to 0<t<1
		double speed = 0;
		if(specials.GetRawButton(BUT_SHOOTER_OUT)) speed=1;
		else if(specials.GetRawButton(BUT_SHOOTER_IN)) speed=-MAX_IN_SPEED;
		double mult = -specials.GetRawAxis(2);
		mult+=1;
		mult*=0.5;
		//speed*=mult;
		if(specials.GetRawButton(BUT_LOWSHOT_A)||specials.GetRawButton(BUT_LOWSHOT_B))
		{
			speed*=LOWSHOT_SPEED;
		}

		//Set wheels to speed
		shooterA.Set(speed);
		shooterB.Set(speed);

		//Change status of solenoid to push ball into wheels
		UpdateKicker(specials.GetRawButton(BUT_FIRE));

		//Change shooter angle using specials y axis, stop if at limit switch
		double specialsY= specials.GetY();
		if(!angleBottom.Get() && specialsY<0) specialsY=0;
		if(!angleTop.Get() && specialsY>0) specialsY=0;
		ShooterAngleToSpeed(specialsY);


		//--------------------Arm---------------------------
		//Move arm if buttons are pressed, also using throttle lever value calculated above in Shooter
		if(specials.GetRawButton(BUT_ARMMAIN_FW) || specials.GetRawButton(BUT_ARMSEC_FW))
		{
			armMain.Set(mult);
		}
		else if(specials.GetRawButton(BUT_ARMMAIN_RV) || specials.GetRawButton(BUT_ARMSEC_RV))
		{
			armMain.Set(-mult);
		}
		else
		{
			armMain.Set(0);
		}
	}


	/* ---------------------------------------------------------------
	 * 								Test & Disable
	 * ---------------------------------------------------------------
	 */
	void TestPeriodic()
	{
		lw->Run();
	}

	void DisabledPeriodic()
	{
		//Make sure nothing is moving when we start up again
		drive.ArcadeDrive(0.0,0);
		shooterA.Set(0);
		shooterB.Set(0);
		angleMotor.Set(0);
		//kicker.Set(0);

		armMain.Set(0);
		//armSecondary.Set(0);

		//Reset pneumatics (remove if needed)
		shifter.Set(shifter.kReverse);
	}

	/**----------------------------------------------------------------------
	 * 							Autonomous
	 * ----------------------------------------------------------------------
	 */

	//----------------Initialization----------------
	void AutonomousInit()
	{
		//Auto state determines where we are in sequence, start at 0
		autoState = 0;

		drive.ArcadeDrive(0.0,0);

		//Get main auto mode
		autoSelected = *((std::string*)autoMode->GetSelected());

		//Get position we are lined up with
		std::string pos = *((std::string*)toBreach->GetSelected());
		if(pos==BREACH_POS_LB) breachPos=-1;
		if(pos==BREACH_POS_0) breachPos=0;
		if(pos==BREACH_POS_1) breachPos=1;
		if(pos==BREACH_POS_2) breachPos=2;
		if(pos==BREACH_POS_3) breachPos=3;

		//Get what defense is in each position
		for(int j=0;j<4;j++)
		{
			defense[j] = *((std::string*)def[j]->GetSelected());
		}

		ZeroShooter();

		//Re zero NavX in case it drifted while stationary
		//NOTE: THIS IS NOT RECALIBRATION.
		ahrs->ResetDisplacement();
		ahrs->ZeroYaw();

		//Re zero all encoders because apparently they drift a lot
		leftEnc.Reset();
		rightEnc.Reset();
		armMain.SetEncPosition(0);
	}



	//-------------------Switchers-------------------------
	void AutonomousPeriodic()
	{
		if(autoSelected==AUTO_MODE_BREACH || autoSelected==AUTO_MODE_FULL || autoSelected==AUTO_MODE_DOUBLELB)
		{
			//--------------------------Full Auto--------------------------
			//Switch case based on current autoState
			switch(autoState)
			{
			case 0: {
				if(breachPos!=-1) AutonomousRaise();
				autoState++;
				break;
			}
			case 1: {

				//Breach functions are timed, run once only
				Breach(breachPos);
				autoState++;
				break;
			}
			case 2:
			{
				if(autoSelected==AUTO_MODE_BREACH)
				{
					autoState=7;
					break;
				}
				//AutonomousClearDefense();
				AutonomousRaise();
				autoState++;
				break;
			}
			case 3:
			{
				if(!AutonomousPerRotateToAngle()) break;
				autoState++;
				break;
			}
			case 4:
			{
				//Confirm & adjust using vision
				if(!AutoAim()) break;
				autoState++;
				break;
			}
			case 5:
			{
				//Fire
				AutonomousFire();
				autoState++;
				break;
			}
			case 6:
			{
				if(autoSelected!=AUTO_MODE_DOUBLELB)
				{
					autoState++;
					break;
				}
				BreachLowBarReverse();
				autoState++;
				break;
			}
			case 7:
			{
				DisabledPeriodic();
				break;
			}
			}
		}

		//-------------------Testing Functions------------------------
		else if(autoSelected==AUTO_MODE_APPROACH_RV)
		{
			//Breach only
			switch(autoState)
			{
			case 0: {
				CorrectedApproach(-1,180);
				autoState++;
				break;
			}
			case 1: {
				//Done
				drive.ArcadeDrive(0.0,0);
				break;
			}
			}
		}
		else if(autoSelected==AUTO_MODE_ROTATETEST)
		{
			//Breach only
			switch(autoState)
			{
			case 0: {
				if(!AutonomousPerRotateToAngle()) break;
				autoState++;
				break;
			}
			case 1: {
				//Done
				drive.ArcadeDrive(0.0,0);
				break;
			}
			}
		}
		else if(autoSelected==AUTO_MODE_VISIONTEST)
		{
			//Breach only
			switch(autoState)
			{
			case 0: {
				if(!AutoAim()) break;
				autoState++;
				break;
			}
			case 1: {
				//Done
				drive.ArcadeDrive(0.0,0);
			}
			}
		}
		else if(autoSelected==AUTO_MODE_FIRETEST)
		{
			//Breach only
			switch(autoState)
			{
			case 0: {
				AutonomousFire();
				autoState++;
				break;
			}
			case 1: {
				//Done
				drive.ArcadeDrive(0.0,0);
				break;
			}
			}
		}
		else if(autoSelected==AUTO_MODE_RAISETEST)
		{
			//Breach only
			switch(autoState)
			{
			case 0: {
				AutonomousRaise();
				autoState++;
				break;
			}
			case 1: {
				//Done
				drive.ArcadeDrive(0.0,0);
				break;
			}
			}
		}
		else if (autoSelected==AUTO_MODE_STRAIGHTTEST)
		{
			//Drive extremely straight
			switch(autoState)
			{
			case 0:
			{
				AutonomousRaise();
				autoState++;
				break;
			}
			case 1:
			{
				CorrectedDrive(0.5,0,3);
				autoState++;
				break;
			}
			case 2:
			{
				//done
				drive.ArcadeDrive(0.,0);
				break;
			}
			}
		}
	}

	//Switcher: call to activate breach type
	//Calls the appropriate function for breaching a given position
	void Breach(int toBreachPos)
	{
		if(toBreachPos==-1)
		{
			BreachLowBar();
		}
		if(defense[toBreachPos]==AUTO_DEFTYPE_PORT){
			BreachPortcullis();
		}
		if(defense[toBreachPos]==AUTO_DEFTYPE_CHEV){
			BreachCheval();
		}
		if(defense[toBreachPos]==AUTO_DEFTYPE_MOAT){
			BreachMoat();
		}
		if(defense[toBreachPos]==AUTO_DEFTYPE_RAMPARTS){
			BreachRamparts();
		}
		if(defense[toBreachPos]==AUTO_DEFTYPE_BRIDGE){
			BreachDrawbridge();
		}
		if(defense[toBreachPos]==AUTO_DEFTYPE_SALLY){
			BreachSally();
		}
		if(defense[toBreachPos]==AUTO_DEFTYPE_RW){
			BreachRockWall();
		}
		if(defense[toBreachPos]==AUTO_DEFTYPE_RT){
			BreachRoughTerrain();
		}
	}


	//---------------------Auto Functions (like Commands)---------------

	bool AutonomousPerRotateToAngle()
	{
		//Rotate to face tower
		//Approx -30, -10, 20, 40
		switch(breachPos)
		{
		case -1:
		{
			if(!RotateToAngle(40)) return false;
			break;
		}
		case 0:
		{
			if(!RotateToAngle(30)) return false;
			break;
		}
		case 1:
		{
			if(!RotateToAngle(10)) return false;
			break;
		}
		case 2:
		{
			if(!RotateToAngle(-10)) return false;
			break;
		}
		case 3:
		{
			if(!RotateToAngle(-20)) return false;
			break;
		}
		}
		return true;
	}

	void AutonomousRaise()
	{
		//Initial firing sequence
		//Raise angle up
		ShooterToAngle(1200);
	}

	void AutonomousFire()
	{
		shooterA.Set(1);
		shooterB.Set(1);
		ShooterAngleToSpeed(0);
		//Let motors spin up
		Wait(1.5);

		//Fire
		UpdateKicker(true);
		while(!UpdateKicker(false)){}

		shooterA.Set(0);
		shooterB.Set(0);
	}

	//---------------------Breach Functions------------------------------
	void BreachPortcullis()
	{
		DriverStation::ReportError("Breaching Portcullis");
		//Turn around
		ArmToAngle(2900);
		CorrectedApproach(-1,180);
		//Deploy arm
		//Back up
		CorrectedDrive(-1,180,4);
		//Reset arm
		ArmToAngle(100);
	}

	void BreachCheval()
	{
		DriverStation::ReportError("Breaching Cheval");
		CorrectedApproach(1,0);
		ShooterToAngle(50);
		CorrectedDrive(0.6,0,4);
	}

	void BreachMoat()
	{
		DriverStation::ReportError("Breaching Moat");
		CorrectedApproach(1,0);
		//Just drive?
		CorrectedDrive(1,0,3);
	}

	void BreachRamparts()
	{
		DriverStation::ReportError("Breaching Ramparts");
		//Just drive?
		CorrectedApproach(1,0);
		CorrectedDrive(1,0,2.8);
	}

	void BreachDrawbridge()
	{
		DriverStation::ReportError("Reaching Drawbridge (Cannot breach)");
		CorrectedApproach(-1,180);
	}

	void BreachSally()
	{
		DriverStation::ReportError("Reaching Sally Port");
		CorrectedApproach(-1,180);
	}

	void BreachRockWall()
	{
		DriverStation::ReportError("Breaching Rock Wall");
		//Just drive?
		CorrectedApproach(1,0);
		drive.ArcadeDrive(1.,0);
		while(ahrs->GetRoll()<60){}
		ShooterToAngle(10);
		Wait(1);
		drive.ArcadeDrive(0.,0);
	}

	void BreachRoughTerrain()
	{
		DriverStation::ReportError("Breaching Rough Terrain");
		//Just drive?
		drive.ArcadeDrive(1.,0);
		Wait(0.1);
		shifter.Set(shifter.kForward);
		CorrectedApproach(1,0);
		CorrectedDrive(1,0,2);
		shifter.Set(shifter.kReverse);
	}

	void BreachLowBar()
	{
		DriverStation::ReportError("Breaching Low Bar");
		//ShooterToAngle(280);//Was 180
		CorrectedApproach(1,0);
		CorrectedDrive(1,0,2.7);
	}

	void BreachLowBarReverse()
	{
		ShooterToAngle(-100);
		angleMotor.Set(-1);
		Wait(0.75);
		angleMotor.Set(0);
		while(!RotateToAngle(0)){}
		CorrectedApproach(-1,0);
		CorrectedDrive(-0.9,0,2.5);

	}
	/*----------------------------------------------------------------------
	 * 							NavX-MXP Navigation
	 * ---------------------------------------------------------------------
	 */

	bool RotateToAngle(double targetAngle, double speed = 0.8)
	{
		//Called periodically, returns true if facing already

		//Get angle from NavX
		double currentAngle = ahrs->GetYaw();

		//If at angle already, stop and return
		//if(currentAngle>targetAngle-ANGLE_TOLERANCE && currentAngle<targetAngle+ANGLE_TOLERANCE)
		if(abs(currentAngle-targetAngle)<ANGLE_TOLERANCE ||
				((targetAngle>180-ANGLE_TOLERANCE || targetAngle<-180+ANGLE_TOLERANCE) &&
						(currentAngle>180-ANGLE_TOLERANCE || currentAngle<-180+ANGLE_TOLERANCE)))
		{
			//Extra logic is to deal with turning to +-180
			drive.ArcadeDrive(0.,0);
			return true;  //true=proceed to next action
		}

		//Not at angle, go to it
		if(currentAngle>targetAngle)
		{
			//Turn left
			drive.ArcadeDrive(0.0,-speed);
		}
		else
		{
			//Turn right
			drive.ArcadeDrive(0.0,speed);
		}
		return false;
	}

	void CorrectedApproach(double speed, double angle)
	{
		while(!RotateToAngle(angle)&&ShouldBeBreaching()){}
		timer.Reset();
		timer.Start();
		while(ahrs->GetRoll()>-6 &&ahrs->GetRoll()&& ShouldBeBreaching())
		{
			//We have not driven far enough, drive

			//Set a correction factor using the sine of our angle difference
			double angleOffset = ahrs->GetYaw()-angle;
			//sin will return a value equivalent to -1 at -90 deg, 0 at 0 deg, and 1 at 90 deg
			//sin wants a value in radians, convert degrees to radians
			//M_PI is the value of pi
			angleOffset = angleOffset * M_PI/180.0;

			double correctionFactor = -sin(angleOffset);
			double speedMult = speed*cos(angleOffset);

			//Use correctionFactor as a turning argument for ArcadeDrive, also slow down if turned a lot
			drive.ArcadeDrive(speedMult,correctionFactor*10);

		}
		timer.Stop();

		drive.ArcadeDrive(0.,0);

	}


	//This periodic function will drive the robot at an angle, correcting as needed
	void CorrectedDrive(double speed, double angle, double numSeconds)
	{
		timer.Reset();
		timer.Start();
		while(timer.Get()<numSeconds && ShouldBeBreaching())
		{
			//We have not driven far enough, drive

			//Set a correction factor using the sine of our angle difference
			double angleOffset = ahrs->GetYaw()-angle;
			//sin will return a value equivalent to -1 at -90 deg, 0 at 0 deg, and 1 at 90 deg
			//sin wants a value in radians, convert degrees to radians
			//M_PI is the value of pi
			angleOffset = angleOffset * M_PI/180.0;

			double correctionFactor = -sin(angleOffset);
			double speedMult = speed*cos(angleOffset);

			//Use correctionFactor as a turning argument for ArcadeDrive, also slow down if turned a lot
			drive.ArcadeDrive(speedMult,correctionFactor*10);

		}
		timer.Stop();

		drive.ArcadeDrive(0.,0);

	}




	/* ----------------------------------------------------------------------
	 * 							Image Processing
	 * ----------------------------------------------------------------------
	 */

	bool AutoAimHardLoop()
	{
		while(!AutoAim() && (specials.GetRawButton(BUT_AUTOAIMA) || IsAutonomous())){};
		return true;
	}

	bool AutoAim()
	{
		//Process image, storing origin in screenPosX and screenPosY
		Vision();

		if(screenPosX==-1&&screenPosY==-1)
		{
			drive.ArcadeDrive(0.,-0.9);
			return false;
		}

		//Check if target is in correct spot (within tolerance) already
		if(screenPosX>TARGET_ORIGIN_X-ORIGIN_X_TOL && screenPosX<TARGET_ORIGIN_X+ORIGIN_X_TOL && screenPosY>TARGET_ORIGIN_Y-ORIGIN_Y_TOL && screenPosY<TARGET_ORIGIN_Y+ORIGIN_Y_TOL)
		{
			drive.ArcadeDrive(0.0,0.0);
			ShooterAngleToSpeed(0);
			return true;  //true=proceed to next action
		}
		else
		{
			//We need to adjust
			//Check L/R
			if(screenPosX>TARGET_ORIGIN_X-ORIGIN_X_TOL && screenPosX<TARGET_ORIGIN_X+ORIGIN_X_TOL)
			{
				//L/R is within tolerance
				//Need up/down adjustment
				drive.ArcadeDrive(0.0,0.0);
				if(screenPosY<TARGET_ORIGIN_Y)
				{
					//Decrease angle
					ShooterAngleToSpeed(0.18);
				}
				else
				{
					//Increase angle
					ShooterAngleToSpeed(-0.09);
				}
			}
			else
			{
				//Need L/R adjustment before proceeding
				ShooterAngleToSpeed(0);
				if(abs(screenPosX-TARGET_ORIGIN_X)<100)
				{
					if(screenPosX>TARGET_ORIGIN_X)
					{
						//Turn right
						drive.ArcadeDrive(0.0,0.48);
						Wait(0.08);
						drive.ArcadeDrive(0.0,0.0);
					}
					else
					{
						//Turn left
						drive.ArcadeDrive(0.0,-0.48);
						Wait(0.08);
						drive.ArcadeDrive(0.0,0.0);
					}
				}
				else
				{
					if(screenPosX>TARGET_ORIGIN_X)
					{
						//Turn right
						drive.ArcadeDrive(0.0,0.45);
					}
					else
					{
						//Turn left
						drive.ArcadeDrive(0.0,-0.45);
					}
				}
			}
		}
		return false;
	}


	void CalibrateVision()//For tuning vision constants only
	{
		RING_HUE_RANGE.minValue = SmartDashboard::GetNumber("Tote hue min", RING_HUE_RANGE.minValue);
		RING_HUE_RANGE.maxValue = SmartDashboard::GetNumber("Tote hue max", RING_HUE_RANGE.maxValue);
		RING_SAT_RANGE.minValue = SmartDashboard::GetNumber("Tote sat min", RING_SAT_RANGE.minValue);
		RING_SAT_RANGE.maxValue = SmartDashboard::GetNumber("Tote sat max", RING_SAT_RANGE.maxValue);
		RING_VAL_RANGE.minValue = SmartDashboard::GetNumber("Tote val min", RING_VAL_RANGE.minValue);
		RING_VAL_RANGE.maxValue = SmartDashboard::GetNumber("Tote val max", RING_VAL_RANGE.maxValue);

		TARGET_ORIGIN_X = SmartDashboard::GetNumber("X To Target",TARGET_ORIGIN_X);
		TARGET_ORIGIN_Y = SmartDashboard::GetNumber("Y To Target",TARGET_ORIGIN_Y);
		ORIGIN_X_TOL = SmartDashboard::GetNumber("X Tol",ORIGIN_X_TOL);
		ORIGIN_Y_TOL = SmartDashboard::GetNumber("Y Tol",ORIGIN_Y_TOL);


		//Retrieve an image from session, store into frame
		IMAQdxGrab(session, frame, true, NULL);
		//Threshold the image looking for ring light color
		imaqError = imaqColorThreshold(binaryFrame, frame, 255, IMAQ_HSV, &RING_HUE_RANGE, &RING_SAT_RANGE, &RING_VAL_RANGE);

		//Count particles in the image
		int numParticles = 0;
		imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);
		SmartDashboard::PutNumber("Masked particles", numParticles);

		//Send masked image to dashboard
		SendToDashboard(binaryFrame, imaqError);

		//filter out small particles
		criteria[0] = {IMAQ_MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100, false, false};
		imaqError = imaqParticleFilter4(binaryFrame, binaryFrame, criteria, 1, &filterOptions, NULL, NULL);

		//Send particle count after filtering for size to dashboard
		imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);
		SmartDashboard::PutNumber("Filtered particles", numParticles);

		//Make sure we even have particles to look at (crash prevention)
		if(numParticles > 0) {
			//Measure particles and sort by particle size
			//Vector is like a dynamic array, it can change size
			//Made of many Struct ParticleReports
			std::vector<ParticleReport> particles;
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
			{
				//Get data from IMAQ and store in par
				ParticleReport par;
				imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_AREA_BY_IMAGE_AREA, &(par.PercentAreaToImageArea));
				imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_AREA, &(par.Area));
				imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_TOP, &(par.BoundingRectTop));
				imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_LEFT, &(par.BoundingRectLeft));
				imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(par.BoundingRectBottom));
				imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_RIGHT, &(par.BoundingRectRight));

				//Add par to the vector
				particles.push_back(par);
			}

			//Sort particles by area using comparator function CompareParticleSizes
			sort(particles.begin(), particles.end(), CompareParticleSizes);

			//The target will always be the biggest thing on the screen, use that to save time
			ParticleReport best = particles.at(0);

			//Origin of particle is average of edges, update variables accordingly
			screenPosX = (best.BoundingRectRight+best.BoundingRectLeft)/2;
			screenPosY = (best.BoundingRectBottom+best.BoundingRectTop)/2;

			SmartDashboard::PutNumber("Target X", screenPosX);
			SmartDashboard::PutNumber("Target Y", screenPosY);


		} else {
			//Somehow, there were no particles on the screen. This exists to stop a crash (vecotor index out of bounds)
		}
	}

	void Vision()
	{
		//Retrieve an image from session, store into frame
		IMAQdxGrab(session, frame, true, NULL);
		//Threshold the image looking for ring light color
		imaqError = imaqColorThreshold(binaryFrame, frame, 255, IMAQ_HSV, &RING_HUE_RANGE, &RING_SAT_RANGE, &RING_VAL_RANGE);

		//Count particles in the image
		int numParticles = 0;
		//filter out small particles
		criteria[0] = {IMAQ_MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100, false, false};
		imaqError = imaqParticleFilter4(binaryFrame, binaryFrame, criteria, 1, &filterOptions, NULL, NULL);

		//Get array of particles
		imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);

		//Make sure we even have particles to look at (crash prevention)
		if(numParticles > 0) {
			int bestIndex = -1;
			double bestArea=0;
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
			{
				double newParticleArea;
				imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_AREA_BY_IMAGE_AREA, &newParticleArea);
				if(newParticleArea>bestArea)
				{
					//Store index and area
					bestArea=newParticleArea;
					bestIndex=particleIndex;
				}
			}

			//imaqMeasureParticle(binaryFrame, bestIndex, 0, IMAQ_MT_AREA_BY_IMAGE_AREA, &(best.PercentAreaToImageArea));
			//imaqMeasureParticle(binaryFrame, bestIndex, 0, IMAQ_MT_AREA, &(best.Area));
			imaqMeasureParticle(binaryFrame, bestIndex, 0, IMAQ_MT_BOUNDING_RECT_TOP, &(best.BoundingRectTop));
			imaqMeasureParticle(binaryFrame, bestIndex, 0, IMAQ_MT_BOUNDING_RECT_LEFT, &(best.BoundingRectLeft));
			imaqMeasureParticle(binaryFrame, bestIndex, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(best.BoundingRectBottom));
			imaqMeasureParticle(binaryFrame, bestIndex, 0, IMAQ_MT_BOUNDING_RECT_RIGHT, &(best.BoundingRectRight));

			//Origin of particle is average of edges, update variables accordingly
			screenPosX = (best.BoundingRectRight+best.BoundingRectLeft)/2;
			screenPosY = (best.BoundingRectBottom+best.BoundingRectTop)/2;

			//SmartDashboard::PutNumber("Target X", screenPosX);
			//SmartDashboard::PutNumber("Target Y", screenPosY);


		} else {

			screenPosX=-1;
			screenPosY=-1;
			//Somehow, there were no particles big enough on the screen. This exists to stop a crash.
			//drive.ArcadeDrive(0.0,0.7);
		}
	}


	//Send the filtered image to the dashboard if there isn't an error
	void SendToDashboard(Image *image, int error)
	{
		if(error < ERR_SUCCESS) {
			DriverStation::ReportError("[ERROR]Send To Dashboard error: " + std::to_string((long)imaqError) + "\n");
		} else {
			CameraServer::GetInstance()->SetImage(binaryFrame);
		}
	}

	//Comparator function for sorting particles. Returns true if particle 1 is larger
	static bool CompareParticleSizes(ParticleReport particle1, ParticleReport particle2)
	{
		//we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	void SetUpCamera()
	{
		if(!DriverStation::GetInstance().IsFMSAttached())
		{
			CAMERA_BRIGHTNESS_AUTO=SmartDashboard::GetNumber("BrightnessTarget",-99);
			CAMERA_WHITEBALANCE=(unsigned int)SmartDashboard::GetNumber("WhiteBalanceTarget",-99);
			CAMERA_EXPOSURE=SmartDashboard::GetNumber("ExposureTarget",-99);
			std::cout << CAMERA_BRIGHTNESS_AUTO << " " << CAMERA_WHITEBALANCE << " " << CAMERA_EXPOSURE << std::endl;
		}
		//Wait(9);
		IMAQdxSetAttribute(session,ATTR_BR_MODE,IMAQdxValueType::IMAQdxValueTypeString,"Manual");
		//Wait(1);
		IMAQdxSetAttribute(session,ATTR_BR_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,CAMERA_BRIGHTNESS_AUTO);
		//Wait(1);
		IMAQdxSetAttribute(session,ATTR_WB_MODE,IMAQdxValueType::IMAQdxValueTypeString,"Manual");
		//Wait(1);
		IMAQdxSetAttribute(session,ATTR_WB_VALUE,IMAQdxValueType::IMAQdxValueTypeU32,CAMERA_WHITEBALANCE);
		//Wait(1);
		IMAQdxSetAttribute(session,ATTR_EX_MODE,IMAQdxValueType::IMAQdxValueTypeString,"Manual");
		//Wait(1);
		IMAQdxSetAttribute(session,ATTR_EX_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,CAMERA_EXPOSURE);
		//Wait(1);
		/*double value;
		IMAQdxGetAttributeMaximum(session,ATTR_EX_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,(void*)&value);
		std::cout << "MAXEX" << value << std::endl;
		IMAQdxGetAttributeMinimum(session,ATTR_EX_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,(void*)&value);
		std::cout << "MINEX" << value << std::endl;
		IMAQdxGetAttributeMaximum(session,ATTR_BR_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,(void*)&value);
		std::cout <<"MAXBR"<< value << std::endl;
		IMAQdxGetAttributeMinimum(session,ATTR_BR_VALUE,IMAQdxValueType::IMAQdxValueTypeF64,(void*)&value);
		std::cout <<"MINBR"<< value << std::endl;
		int val2 = -111;
		IMAQdxGetAttributeMaximum(session,ATTR_WB_VALUE,IMAQdxValueType::IMAQdxValueTypeU32,(void*)&val2);
		std::cout <<"MAXWB"<< val2 << std::endl;
		IMAQdxGetAttributeMinimum(session,ATTR_WB_VALUE,IMAQdxValueType::IMAQdxValueTypeU32,(void*)&val2);
		std::cout <<"MINWB"<< val2 << std::endl;*/
	}

	/* -------------------------------------------------------------------
	 * 							Encoder Stuff
	 * -------------------------------------------------------------------
	 */

 	void ShooterToAngle(int target)
	{
		target=-target;
		if(target<angleMotor.GetEncPosition())
		{
			ShooterAngleToSpeed(0.5);
			while(target<angleMotor.GetEncPosition() && angleTop.Get() && ShouldBeBreaching()){};
		}
		else if(target>angleMotor.GetEncPosition())
		{
			ShooterAngleToSpeed(-0.5);
			while(target>angleMotor.GetEncPosition() && angleBottom.Get() && ShouldBeBreaching()){};
		}
		ShooterAngleToSpeed(0);
	}

	void ZeroShooter()
	{
		ShooterAngleToSpeed(-0.5);
		while(angleBottom.Get() && IsEnabled() && IsAutonomous()){}
		ShooterAngleToSpeed(0);
		angleMotor.SetEncPosition(0);
	}
	void ShooterAngleToSpeed(double percent)
	{
		SmartDashboard::PutNumber("Percent",abs(percent*100)/100.0);
		if(abs(percent*100)/100.0<SHOOTER_ANGLE_HOLD_PERCENT)
		{
			angleMotor.Set(SHOOTER_ANGLE_HOLD_PERCENT);
		}
		else
		{
			//This block smoothes shooter movement outside the holding range, graph as piecewise function to understand
			if(percent>0)
			{
				angleMotor.Set((1-SHOOTER_ANGLE_HOLD_PERCENT)/(1-SHOOTER_ANGLE_HOLD_PERCENT)*percent
						+(1-SHOOTER_ANGLE_HOLD_PERCENT)/(1-SHOOTER_ANGLE_HOLD_PERCENT)*SHOOTER_ANGLE_HOLD_PERCENT
						+SHOOTER_ANGLE_HOLD_PERCENT);
			}
			else
			{
				angleMotor.Set((-1-SHOOTER_ANGLE_HOLD_PERCENT)/(-1+SHOOTER_ANGLE_HOLD_PERCENT)*percent
						+(-1-SHOOTER_ANGLE_HOLD_PERCENT)/(-1+SHOOTER_ANGLE_HOLD_PERCENT)*SHOOTER_ANGLE_HOLD_PERCENT
						+SHOOTER_ANGLE_HOLD_PERCENT);
			}
		}
	}


	void ArmToAngle(int target)
	{
		if(target<armMain.GetEncPosition())
		{
			armMain.Set(-1);
			while(target<armMain.GetEncPosition() && IsEnabled()){};

		}
		else if(target>armMain.GetEncPosition())
		{
			armMain.Set(1);
			while(target>armMain.GetEncPosition() && IsEnabled()){};
		}
		armMain.Set(0);
	}


	/* ----------------------------------------------------
	 *                   Misc
	 * ----------------------------------------------------
	 */

	bool ShouldBeBreaching()
	{
		//Return true if we are autonomous and enabled OR a button is pressed and we are enabled
		return (IsAutonomous()&&IsEnabled())||(mainStick.GetRawButton(BUT_BREACH2)||mainStick.GetRawButton(BUT_BREACH3)||mainStick.GetRawButton(BUT_BREACH4)||mainStick.GetRawButton(BUT_BREACH5)||mainStick.GetRawButton(BUT_APPROACH));
	}

	bool UpdateKicker(bool activate)
	{
		if(kickerMoving)
		{
			//Kicker is moving, check if stop
			int enc = kicker.GetEncPosition();
			if(enc>=KICKER_TICS_PER_REV)
			{
				kicker.Set(0);
				kicker.SetEncPosition(kicker.GetEncPosition()-KICKER_TICS_PER_REV);
				kickerMoving=false;
				return true;
			}
			return false;
		}
		else
		{
			//Kicker is not moving
			if(activate)
			{
				kickerMoving=true;
				kicker.Set(1);
			}
			return false;
		}
	}
};

START_ROBOT_CLASS(Robot)
