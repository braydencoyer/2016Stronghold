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
	const std::string AUTO_MODE_OFF = "Off";
	const std::string AUTO_MODE_FULL = "Full";
	const std::string AUTO_MODE_BREACHTEST = "BreachTest";
	const std::string AUTO_MODE_APROACHTEST = "ApproachTest";
	const std::string AUTO_MODE_CLEARTEST = "ClearTest";
	const std::string AUTO_MODE_ROTATETEST = "RotateTest";
	const std::string AUTO_MODE_VISIONTEST = "VisionTest";
	const std::string AUTO_MODE_FIRETEST = "FireTest";
	const std::string AUTO_MODE_RAISETEST = "RaiseTest";

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



	//---------------------------NAVX CONSTANTS--------------------
	double ANGLE_TOLERANCE = 2;   //Degrees

	//--------------------------VISION CONSTANTS-------------------
	int TARGET_ORIGIN_X = 250;
	int TARGET_ORIGIN_Y = 250;
	int ORIGIN_X_TOL = 10;
	int ORIGIN_Y_TOL = 10;

	Range RING_HUE_RANGE = {101, 225};	//Default hue range for ring light, old=64
	Range RING_SAT_RANGE = {200, 255};	//Default saturation range for ring light, old=88
	Range RING_VAL_RANGE = {245, 255};	//Default value range for ring light old=230
	float AREA_MINIMUM = 0.5; //Area minimum for particle as a percentage of total image area
	double VIEW_ANGLE = 60; //View angle for camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480

	//Auto Modes
	SendableChooser *autoMode;
	SendableChooser *def[4];
	SendableChooser *toBreach;

	std::string autoSelected;
	std::string defense[4];
	int breachPos;

	int autoState;


	//-----------------------MOTORS-------------------------
	Victor left, right;
	RobotDrive drive;

	CANTalon shooterA;
	CANTalon shooterB;
	CANTalon angleMotor;

	const double SHOOTER_ANGLE_HOLD_PERCENT = 0.1;

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

	//Images
	Image *frame;
	Image *binaryFrame;

	//Filter function arguments
	ParticleFilterCriteria2 criteria[1];
	ParticleFilterOptions2 filterOptions = {0,0,1,1};

	//IMAQ storage
	IMAQdxSession session;
	int imaqError;

	//Position of target on-screen, to be used later.
	double screenPosX;
	double screenPosY;

	//--------------------------PDP----------------------------------
	PowerDistributionPanel pdp;


	//-------------------------PNEUMATICS---------------------------
	//Shifter
	DoubleSolenoid shifter;

	//Hanging
	DoubleSolenoid goingUp;

	//Thing that pushes balls into wheels
	DoubleSolenoid shootyStick;

	Compressor compressor;

	//--------------------------LIMIT SWITCHES--------------------------
	LimitSwitch angleBottom;
	LimitSwitch angleTop;

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
		mainStick(CH_DRIVESTICK),
		specials(CH_SPECIALSTICK),
		pdp(CH_PDP),
		shifter(CH_SHIFTER_PCM,CH_SHIFTER_FW,CH_SHIFTER_RV),
		goingUp(CH_HANG_PCM,CH_HANG_FW,CH_HANG_RV),
		shootyStick(CH_SHOOTSTICK_PCM,CH_SHOOTSTICK_FW,CH_SHOOTSTICK_RV),
		compressor(CH_PCMA),
		angleBottom(CH_SHOOTER_ANGLE_BOTTOM,true),
		angleTop(CH_SHOOTER_ANGLE_TOP,true)
	{

		//--------Motor inversion----------------
		//shooterB.SetControlMode(CANSpeedController::kFollower);
		shooterB.SetInverted(false);
		shooterA.SetInverted(true);
		//shooterB.Set(10);

		angleMotor.SetFeedbackDevice(angleMotor.QuadEncoder);
		angleMotor.SetInverted(true);
		//angleMotor.SetSensorDirection(false);
		//angleMotor.SetControlMode(angleMotor.kPosition);
		//angleMotor.ConfigEncoderCodesPerRev(1475);
		//angleMotor.SetCloseLoopRampRate(0.25);
		//angleMotor.SetPID(0.5,0,0);
	}


	void RobotInit()
	{
		drive.SetSafetyEnabled(false);
		left.SetSafetyEnabled(false);
		right.SetSafetyEnabled(false);

		angleMotor.SetEncPosition(0);

		SmartDashboard::PutNumber("Angle Target:",0);

		SmartDashboard::PutNumber("Encoder Target",0);

		//--------------NAVX-MXP-------------------
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
		autoMode = new SendableChooser();
		autoMode->AddDefault("Auto Off", (void*)&AUTO_MODE_OFF);
		autoMode->AddObject("Full Auto", (void*)&AUTO_MODE_FULL);
		autoMode->AddObject("[TEST]Breach",(void*)&AUTO_MODE_BREACHTEST);
		autoMode->AddObject("[TEST]Approach",(void*)&AUTO_MODE_APROACHTEST);
		autoMode->AddObject("[TEST]Clear",(void*)&AUTO_MODE_CLEARTEST);
		autoMode->AddObject("[TEST]Rotate",(void*)&AUTO_MODE_ROTATETEST);
		autoMode->AddObject("[TEST]Vision",(void*)&AUTO_MODE_VISIONTEST);
		autoMode->AddObject("[TEST]Fire",(void*)&AUTO_MODE_FIRETEST);

		SmartDashboard::PutData("Auto Modes", autoMode);

		toBreach = new SendableChooser();
		toBreach->AddObject("1 (Lowbar)",(void*)&BREACH_POS_LB);
		toBreach->AddDefault("2",(void*)&BREACH_POS_0);
		toBreach->AddObject("3",(void*)&BREACH_POS_1);
		toBreach->AddObject("4",(void*)&BREACH_POS_2);
		toBreach->AddObject("5",(void*)&BREACH_POS_3);

		SmartDashboard::PutData("Breach", toBreach);

		for(int j=0;j<4;j++)
		{
			def[j] = new SendableChooser();
		}

		def[0]->AddDefault("Defense2",(void*)&AUTO_DEFTYPE_RW);
		def[1]->AddDefault("Defense3",(void*)&AUTO_DEFTYPE_RW);
		def[2]->AddDefault("Defense4",(void*)&AUTO_DEFTYPE_RW);
		def[3]->AddDefault("Defense5",(void*)&AUTO_DEFTYPE_RW);

		for(int j=0;j<4;j++)
		{

			//To display which is which (use random def to prevent crash)

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
		//Create image stuff
		binaryFrame = imaqCreateImage(IMAQ_IMAGE_U8,0);
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}
		imaqError = IMAQdxConfigureGrab(session);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
		}

		IMAQdxStartAcquisition(session);


		SmartDashboard::PutNumber("Tote hue min", RING_HUE_RANGE.minValue);
		SmartDashboard::PutNumber("Tote hue max", RING_HUE_RANGE.maxValue);
		SmartDashboard::PutNumber("Tote sat min", RING_SAT_RANGE.minValue);
		SmartDashboard::PutNumber("Tote sat max", RING_SAT_RANGE.maxValue);
		SmartDashboard::PutNumber("Tote val min", RING_VAL_RANGE.minValue);
		SmartDashboard::PutNumber("Tote val max", RING_VAL_RANGE.maxValue);
	}
	/* ------------------------------------------------------------------------
	 * 									Teleop
	 * ------------------------------------------------------------------------
	 */

	void TeleopInit()
	{
		//end = std::chrono::system_clock::now();
	}

	void TeleopPeriodic()
	{

		//start = std::chrono::system_clock::now();
		//std::chrono::duration<double> elapsed_seconds = start-end;
		//end=start;
		//SmartDashboard::PutNumber("Periodic:",elapsed_seconds.count());

		SmartDashboard::PutNumber("Forward Speed",mainStick.GetY());
		SmartDashboard::PutNumber("Angle Motor Percent",specials.GetY());

		//---------------AUTOAIM-------------------
		if(specials.GetRawButton(BUT_AUTOAIMA))
		{
			AutoAimHardLoop();
			return;
		}
		//This is for mask tuning only
		if(specials.GetRawButton(BUT_AUTOAIMB))
		{
			VisionStuff();
			return;
		}

		//---------------------Debug outs & testing functions, disable for comp-------------
		SmartDashboard::PutNumber("Encoder Val:",angleMotor.GetEncPosition());

		if(mainStick.GetRawButton(3))
		{
			double tA = SmartDashboard::GetNumber("Angle Target:",0);

			RotateToAngle(tA);
			return;
		}

		if(mainStick.GetRawButton(4))
		{
			ahrs->ResetDisplacement();
			ahrs->ZeroYaw();
		}

		if(mainStick.GetRawButton(6))
		{
			angleMotor.SetEncPosition(0);
		}

		//------------------AUTOBREACH----------------------
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
		IMAQdxGrab(session, frame, true, NULL);
		imaqDrawShapeOnImage(frame, frame, { TARGET_ORIGIN_X, TARGET_ORIGIN_Y,ORIGIN_X_TOL,ORIGIN_Y_TOL}, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_RECT, 0.0f);
		CameraServer::GetInstance()->SetImage(frame);

		//-----------------DRIVE SYSTEM---------------------
		drive.ArcadeDrive(mainStick);

		//-----------------SHIFTER--------------------------
		if(mainStick.GetRawButton(BUT_SHIFTER))
			shifter.Set(shifter.kForward);
		else
			shifter.Set(shifter.kReverse);

		//-----------------HANGING-------------------------
		if(mainStick.GetRawButton(BUT_HANG))
		{
			goingUp.Set(goingUp.kForward);
		}
		else
		{
			goingUp.Set(goingUp.kReverse);
		}

		//-------------------SHOOTER------------------

		//Speed based on throttle lever, convert from -1<t<1 to 0<t<1
		double speed = 0;
		if(specials.GetRawButton(BUT_REVUP)) speed=1;
		else if(specials.GetRawButton(BUT_SUCKIN)) speed=-1;
		double mult = -specials.GetRawAxis(2);
		mult+=1;
		mult*=0.5;
		speed*=mult;

		//Set wheels to speed
		shooterA.Set(speed);
		shooterB.Set(speed);

		//Change status of solenoid to push ball into wheels
		if(specials.GetRawButton(BUT_FIRE))
		{
			shootyStick.Set(shootyStick.kForward);
		}
		else
		{
			shootyStick.Set(shootyStick.kReverse);
		}

		//Change shooter angle
		double specialsY= specials.GetY();
		//Move encoder to target TODO Remove when done
		if(mainStick.GetRawButton(5))
		{
			ShooterToAngle(SmartDashboard::GetNumber("Encoder Target",0));
		}
		if(!angleBottom.Get() && specialsY<0) specialsY=0;
		if(!angleTop.Get() && specialsY>0) specialsY=0;
		ShooterAngleToSpeed(specialsY);


		//std::chrono::duration<double> elapsed_seconds1 = std::chrono::system_clock::now()-start;
		//SmartDashboard::PutNumber("Code Time:",elapsed_seconds1.count());

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

		//Reset pneumatics (remove if needed)
		shootyStick.Set(shootyStick.kReverse);
		goingUp.Set(goingUp.kReverse);
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

		//Reset NavX in case it drifted while stationary
		ahrs->ResetDisplacement();
		ahrs->ZeroYaw();

	}



	//-------------------Switchers-------------------------
	void AutonomousPeriodic()
	{
		if(autoSelected==AUTO_MODE_FULL)
		{
			//--------------------------Full Auto--------------------------
			//Switch case based on current autoState
			switch(autoState)
			{
			case 0: {
				AutonomousApproach();
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
				//Drive to position on arc around tower
				//TODO Write with wait statements
				AutonomousClearDefense();
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
				AutonomousRaise();
				autoState++;
				break;
			}
			case 5:
			{
				//Confirm & adjust using vision
				if(!AutoAim()) break;
				autoState++;
				break;
			}
			case 6:
			{
				//Fire
				AutonomousFire();
				autoState++;
				break;
			}
			case 7:
			{
				//done
				DisabledPeriodic();
				break;
			}
			}
		}

		//-------------------Testing Functions------------------------
		else if(autoSelected==AUTO_MODE_BREACHTEST)
		{
			//Breach only
			switch(autoState)
			{
			case 0: {
				Breach(breachPos);
				autoState++;
				break;
			}
			case 1: {
				//Done
				drive.ArcadeDrive(0.0,0);
			}
			}
		}
		else if(autoSelected==AUTO_MODE_APROACHTEST)
		{
			//Breach only
			switch(autoState)
			{
			case 0: {
				AutonomousApproach();
				autoState++;
				break;
			}
			case 1: {
				//Done
				drive.ArcadeDrive(0.0,0);
			}
			}
		}
		else if(autoSelected==AUTO_MODE_CLEARTEST)
		{
			//Breach only
			switch(autoState)
			{
			case 0: {
				AutonomousClearDefense();
				autoState++;
				break;
			}
			case 1: {
				//Done
				drive.ArcadeDrive(0.0,0);
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
			}
			}
		}
	}

	//Switcher: call to activate breach type
	//Calls the appropriate function for breaching a given position
	void Breach(int toBreachPos)
	{
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

	void AutonomousApproach()
	{
		drive.ArcadeDrive(1.0,0);
		Wait(1);
		drive.ArcadeDrive(0.0,0);
	}

	void AutonomousClearDefense()
	{
		drive.ArcadeDrive(1.0,0);
		Wait(1);
		drive.ArcadeDrive(0.0,0);
	}

	bool AutonomousPerRotateToAngle()
	{
		//Rotate to face tower TODO Calculate angles to face tower
		//Approx -30, -10, 20, 40
		switch(breachPos)
		{
		case 0:
		{
			if(!RotateToAngle(40)) return false;
			break;
		}
		case 1:
		{
			if(!RotateToAngle(20)) return false;
			break;
		}
		case 2:
		{
			if(!RotateToAngle(-10)) return false;
			break;
		}
		case 3:
		{
			if(!RotateToAngle(-30)) return false;
			break;
		}
		}
		return true;
	}

	void AutonomousRaise()
	{
		//Initial firing sequence
		//Raise angle up
		//TODO Figure out angle
		ShooterToAngle(500);
	}


	void AutonomousFire()
	{
		shooterA.Set(0.25);
		shooterB.Set(0.25);
		//Let motors spin up
		Wait(1);

		//Fire
		shootyStick.Set(shootyStick.kForward);
		Wait(1);
		shootyStick.Set(shootyStick.kReverse);
		shooterA.Set(0);
		shooterB.Set(0);
	}

	//---------------------Breach Functions------------------------------
	void BreachPortcullis()
	{
		DriverStation::ReportError("Breaching Portcullis");
	}

	void BreachCheval()
	{
		DriverStation::ReportError("Breaching Cheval");
	}

	void BreachMoat()
	{
		DriverStation::ReportError("Breaching Moat");
		//Just drive?
	}

	void BreachRamparts()
	{
		DriverStation::ReportError("Breaching Ramparts");
		//Just drive?
	}

	void BreachDrawbridge()
	{
		DriverStation::ReportError("Breaching Drawbridge");
	}

	void BreachSally()
	{
		DriverStation::ReportError("Breaching Sally Port");
	}

	void BreachRockWall()
	{
		DriverStation::ReportError("Breaching Rock Wall");
		//Just drive?
	}

	void BreachRoughTerrain()
	{
		DriverStation::ReportError("Breaching Rough Terrain");
		//Just drive?
	}

	/*----------------------------------------------------------------------
	 * 							NavX-MXP Navigation
	 * ---------------------------------------------------------------------
	 */

	bool RotateToAngle(double targetAngle)
	{
		//Called periodically, returns true if facing already

		//Get angle from NavX
		double currentAngle = ahrs->GetYaw();

		//If at angle already, stop and return
		//if(currentAngle>targetAngle-ANGLE_TOLERANCE && currentAngle<targetAngle+ANGLE_TOLERANCE)
		if(abs(currentAngle-targetAngle)<ANGLE_TOLERANCE)
		{
			return true;  //true=proceed to next action
		}

		//Not at angle, go to it
		if(currentAngle>targetAngle)
		{
			//Turn left
			drive.ArcadeDrive(0.0,0.5);
		}
		else
		{
			//Turn right
			drive.ArcadeDrive(0.0,-0.5);
		}
		return false;
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
		VisionStuffFaster();

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
					ShooterAngleToSpeed(-0.3);
				}
				else
				{
					//Increase angle
					ShooterAngleToSpeed(0.3);
				}
			}
			else
			{
				//Need L/R adjustment before proceeding
				if(abs(screenPosX-TARGET_ORIGIN_X)<100)
				{
					if(screenPosX>TARGET_ORIGIN_X)
					{
						//Turn right
						drive.ArcadeDrive(0.0,-0.45);
						Wait(0.1);
						drive.ArcadeDrive(0.0,0.0);
					}
					else
					{
						//Turn left
						drive.ArcadeDrive(0.0,0.45);
						Wait(0.1);
						drive.ArcadeDrive(0.0,0.0);
					}
				}
				else
				{
					if(screenPosX>TARGET_ORIGIN_X)
					{
						//Turn right
						drive.ArcadeDrive(0.0,-0.45);
					}
					else
					{
						//Turn left
						drive.ArcadeDrive(0.0,0.45);
					}
				}
			}
		}
		return false;
	}


	void VisionStuff()//For tuning vision constants only
	{
		RING_HUE_RANGE.minValue = SmartDashboard::GetNumber("Tote hue min", RING_HUE_RANGE.minValue);
		RING_HUE_RANGE.maxValue = SmartDashboard::GetNumber("Tote hue max", RING_HUE_RANGE.maxValue);
		RING_SAT_RANGE.minValue = SmartDashboard::GetNumber("Tote sat min", RING_SAT_RANGE.minValue);
		RING_SAT_RANGE.maxValue = SmartDashboard::GetNumber("Tote sat max", RING_SAT_RANGE.maxValue);
		RING_VAL_RANGE.minValue = SmartDashboard::GetNumber("Tote val min", RING_VAL_RANGE.minValue);
		RING_VAL_RANGE.maxValue = SmartDashboard::GetNumber("Tote val max", RING_VAL_RANGE.maxValue);

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

	void VisionStuffFaster()
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

		//Send particle count after filtering for size to dashboard
		imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);

		//Make sure we even have particles to look at (crash prevention)
		if(numParticles > 0) {
			ParticleReport best;
			best.Area=0;
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
			{
				double newParticleArea;
				double oldParticleArea = best.Area;
				imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_AREA, &newParticleArea);
				if(newParticleArea>oldParticleArea)
				{
					//Replace particle in memory
					imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_AREA_BY_IMAGE_AREA, &(best.PercentAreaToImageArea));
					imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_AREA, &(best.Area));
					imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_TOP, &(best.BoundingRectTop));
					imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_LEFT, &(best.BoundingRectLeft));
					imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(best.BoundingRectBottom));
					imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_RIGHT, &(best.BoundingRectRight));
				}
			}
			//Origin of particle is average of edges, update variables accordingly
			screenPosX = (best.BoundingRectRight+best.BoundingRectLeft)/2;
			screenPosY = (best.BoundingRectBottom+best.BoundingRectTop)/2;

			SmartDashboard::PutNumber("Target X", screenPosX);
			SmartDashboard::PutNumber("Target Y", screenPosY);


		} else {
			//Somehow, there were no particles big enough on the screen. This exists to stop a crash.
			drive.ArcadeDrive(0.0,0.7);
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

	/* -------------------------------------------------------------------
	 * 							Encoder Stuff
	 * -------------------------------------------------------------------
	 */

	void ShooterToAngle(int target)
	{
		if(target<angleMotor.GetEncPosition())
		{
			ShooterAngleToSpeed(-0.5);
			while(target<angleMotor.GetEncPosition() && angleBottom.Get() && IsEnabled()){};
		}
		else if(target>angleMotor.GetEncPosition())
		{
			ShooterAngleToSpeed(0.5);
			while(target>angleMotor.GetEncPosition() && angleTop.Get() && IsEnabled()){};
		}
	}

	void ZeroShooter()
	{
		ShooterAngleToSpeed(-0.5);
		while(!angleBottom.Get() && IsEnabled() && IsAutonomous()){}
		ShooterAngleToSpeed(0);
		angleMotor.SetEncPosition(0);
	}

	void ShooterAngleToSpeed(double percent)
	{
		if(abs(percent)<SHOOTER_ANGLE_HOLD_PERCENT)
		{
			angleMotor.Set(SHOOTER_ANGLE_HOLD_PERCENT);
		}
		else
		{
			angleMotor.Set(percent);
		}
	}

};

START_ROBOT_CLASS(Robot)
