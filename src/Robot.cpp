#include "WPILib.h"
#include "AHRS.h"

class Robot: public IterativeRobot
{
public:
	LiveWindow *lw = LiveWindow::GetInstance();

	//Out 6
	//In 7
	//Half speed 8

	//Auto Modes
	const std::string AUTO_MODE_OFF = "Off";
	const std::string AUTO_MODE_FULL = "Full";
	const std::string AUTO_MODE_BREACHTEST = "Breach";

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



	//Constants
	double ANGLE_TOLERANCE = 1;   //Degrees
	double XY_TOLERANCE = 0.25;  //Meters

	//Auto Modes
	SendableChooser *autoMode;
	SendableChooser *def[4];
	SendableChooser *toBreach;

	std::string autoSelected;
	std::string defense[4];
	int breachPos;

	int autoState;


	//Drive
	Victor left, right;
	RobotDrive drive;

	//Sensors
	AHRS *ahrs;


	//Human input
	Joystick mainStick;
	Joystick specials;


	//PDP is on 5

	//PCM LOCS
	const static int PCMA = 0;
	const static int PCMB = 1;

	//PDP
	PowerDistributionPanel pdp;


	//Pneumatics
	DoubleSolenoid shifter;

	DoubleSolenoid goingUpA;

	//Shooter
	CANTalon shooter;
	CANTalon shooterB;
	DoubleSolenoid shootyStick;

	Compressor compressor;

	CANTalon angleMotor;
	CANTalon lift;

	/*--------------------------------------------------------------
	 *						Initialization
	 * -------------------------------------------------------------
	 */

	Robot():
		left(1),right(2),drive(left,right),mainStick(0),specials(1),pdp(0),
		shifter(PCMA,0,1),goingUpA(PCMA,2,3)
		,shooter(0),shooterB(1),shootyStick(PCMB,0,1),compressor(PCMA),
		angleMotor(2),lift(3)
	{

		//shooterB.SetControlMode(CANSpeedController::kFollower);
		shooterB.SetInverted(false);
		shooter.SetInverted(true);
		//shooterB.Set(10);

		//angleMotor.SetPositionMode();
	}


	void RobotInit()
	{
		autoMode = new SendableChooser();
		autoMode->AddDefault("Auto Off", (void*)&AUTO_MODE_OFF);
		autoMode->AddObject("Full Auto", (void*)&AUTO_MODE_FULL);
		autoMode->AddObject("Breach Only (TEST)",(void*)&AUTO_MODE_BREACHTEST);
		SmartDashboard::PutData("Auto Modes", autoMode);


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

		toBreach = new SendableChooser();
		toBreach->AddDefault("2",(void*)&BREACH_POS_0);
		toBreach->AddObject("3",(void*)&BREACH_POS_1);
		toBreach->AddObject("4",(void*)&BREACH_POS_2);
		toBreach->AddObject("5",(void*)&BREACH_POS_3);

		SmartDashboard::PutData("Breach", toBreach);

		for(int j=0;j<4;j++)
		{
			def[j] = new SendableChooser();

			//Add each option
			def[j]->AddDefault("Portcullis", (void*)&AUTO_DEFTYPE_PORT);
			def[j]->AddObject("Cheval", (void*)&AUTO_DEFTYPE_CHEV);
			def[j]->AddObject("Moat", (void*)&AUTO_DEFTYPE_MOAT);
			def[j]->AddObject("Ramparts", (void*)&AUTO_DEFTYPE_RAMPARTS);
			def[j]->AddObject("Drawbridge", (void*)&AUTO_DEFTYPE_BRIDGE);
			def[j]->AddObject("Sally Door", (void*)&AUTO_DEFTYPE_SALLY);
			def[j]->AddObject("Rock Wall", (void*)&AUTO_DEFTYPE_RW);
			def[j]->AddObject("Rough Terrain", (void*)&AUTO_DEFTYPE_RT);

			SmartDashboard::PutData("Defense"+j, def[j]);
		}
	}
	/* ------------------------------------------------------------------------
	 * 									Teleop
	 * ------------------------------------------------------------------------
	 */

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		//Wheel drive
		drive.ArcadeDrive(mainStick);

		//Shifter
		if(mainStick.GetRawButton(1))
			shifter.Set(shifter.kForward);
		else
			shifter.Set(shifter.kReverse);

		//Lift
		if(specials.GetRawButton(11))
		{
			goingUpA.Set(goingUpA.kForward);
		}
		else if(specials.GetRawButton(10))
		{
			goingUpA.Set(goingUpA.kReverse);
		}

		double speed = 0;
		if(specials.GetRawButton(6)) speed=1;
		else if(specials.GetRawButton(7)) speed=-1;
		double mult = -specials.GetRawAxis(2);
		mult+=1;
		mult*=0.5;

		speed*=mult;
		shooter.Set(speed);
		shooterB.Set(speed);

		if(specials.GetRawButton(1))
		{
			shootyStick.Set(shootyStick.kForward);
		}
		else
		{
			shootyStick.Set(shootyStick.kReverse);
		}

		//Shooter angle
		angleMotor.Set(specials.GetY());

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
		drive.ArcadeDrive(0.0,0);
		shooter.Set(0);
	}

	/**----------------------------------------------------------------------
	 * 							Autonomous
	 * ----------------------------------------------------------------------
	 */
	void AutonomousInit()
	{
		autoState = 0;
		drive.ArcadeDrive(0.0,0);
		drive.SetSafetyEnabled(false);
		left.SetSafetyEnabled(false);
		right.SetSafetyEnabled(false);
		//autoMode = (SendableChooser*) SmartDashboard::GetData("Auto Modes");
		//DriverStation::ReportError("Getting Auto mode...\n");
		//DriverStation::ReportError(autoMode->GetSelected());
		autoSelected = *((std::string*)autoMode->GetSelected());
		DriverStation::ReportError("Mode: "+autoSelected);

		//toBreach = (SendableChooser*) SmartDashboard::GetData("Breach");
		std::string pos = *((std::string*)toBreach->GetSelected());
		if(pos==BREACH_POS_0) breachPos=0;
		if(pos==BREACH_POS_1) breachPos=1;
		if(pos==BREACH_POS_2) breachPos=2;
		if(pos==BREACH_POS_3) breachPos=3;

		for(int j=0;j<4;j++)
		{
			defense[j] = *((std::string*)def[j]->GetSelected());
			//DriverStation::ReportError(defense[j]);
		}
		DriverStation::ReportError("Done!");

		ahrs->ResetDisplacement();
		ahrs->ZeroYaw();

	}

	void AutonomousPeriodic()
	{
		//DriverStation::ReportError("Looping");
		if(autoSelected=="Full")
		{
			//Do everything
			switch(autoState)
			{
			case 0: {
				//TODO Use move function to move forward ? meters
				if(!NavigateTo(0,1)) return;
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
				//if(!NavigateTo(0,1)) return; TODO Calculate Arc
				break;
			}
			case 3:
			{
				//Initial firing sequence

				break;
			}
			case 4:
			{
				//Confirm & adjust using vision

				break;
			}
			case 5:
			{
				//Fire

				break;
			}
			case 6:
			{
				//done
				drive.ArcadeDrive(0.0,0);
				break;
			}
			}
		}
		else if(autoSelected=="Breach")
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
	}





	/*------------------------------------------------------------------------
	 *                        BREACHING        TODO Write breach protocols
	 * -----------------------------------------------------------------------
	 */

	//Switcher: call to activate breach type
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


	//Functions to breach each type, excluding lowbar

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
	}

	void BreachRamparts()
	{
		DriverStation::ReportError("Breaching Ramparts");
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
	}

	void BreachRoughTerrain()
	{
		DriverStation::ReportError("Breaching Rough Terrain");
	}

	/*----------------------------------------------------------------------
	 * 							NavX-MXP Navigation    TODO Test this
	 * ---------------------------------------------------------------------
	 */

	//IMPORTANT: All accel values ARE displaced by current orientation; as such, so are dispacements!!!
	bool NavigateTo(double xTarget, double yTarget)
	{
		//Get current location (swapped due to 3d to 2d conversion)
		double currentY = ahrs->GetDisplacementX();
		double currentX = ahrs->GetDisplacementY();

		//Get offset
		double xOff = xTarget-currentX;
		double yOff = yTarget-currentY;

		if(abs(xOff)<XY_TOLERANCE && abs(yOff)<XY_TOLERANCE)
		{
			//At or close enough to location, true=proceed to next action
			return true;
		}

		//Not at location yet
		//Turn towoards target, stop this execution if turning
		//Adjust if xOff is exactly 0 to prevent y/0 crash
		if(xOff==0) xOff = 0.0000000000001;
		double angle = atan(abs(yOff)/abs(xOff));

		//If still rotating, return false to halt execution
		//Manually select quadrant bc atan sucks
		if(xOff>0&&yOff>0)
		{
			if(!RotateToAngle(90-angle)) return false;
		}
		else if(xOff<0&&yOff>0)
		{
			if(!RotateToAngle(-(90-angle))) return false;
		}
		else if(xOff>0&&yOff<0)
		{
			if(!RotateToAngle(90+angle)) return false;
		}
		else if(xOff>0&&yOff>0)
		{
			if(!RotateToAngle(-(90+angle))) return false;
		}

		//If we are still here, we are facing in right direction. Onward!
		drive.ArcadeDrive(1,0);

		return false;
	}


	bool RotateToAngle(double targetAngle)
	{
		double currentAngle = ahrs->GetYaw();

		//If at angle already, stop and return
		if(currentAngle>targetAngle-ANGLE_TOLERANCE && currentAngle<targetAngle+ANGLE_TOLERANCE)
		{
			return true;  //true=proceed to next action
		}

		//Not at angle, go to it
		if(currentAngle>targetAngle)
		{
			//Turn left
			drive.ArcadeDrive(0.0,-1);
		}
		else
		{
			//Turn right
			drive.ArcadeDrive(0.0,1);
		}
		return false;
	}





	/* ----------------------------------------------------------------------
	 * 							Image Processing     TODO
	 * ----------------------------------------------------------------------
	 */
};

START_ROBOT_CLASS(Robot)
