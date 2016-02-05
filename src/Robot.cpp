#include "WPILib.h"
#include "AHRS.h"

class Robot: public IterativeRobot
{
public:
	LiveWindow *lw = LiveWindow::GetInstance();

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


	//PCM LOCS
	const static int PCMA = 0;
	const static int PCMB = 1;


	//Pneumatics
	DoubleSolenoid shifter;

	DoubleSolenoid goingUpA, goingUpB;

	//Shooter
	Victor shooter;
	DoubleSolenoid shootyStick;
	DoubleSolenoid shooterAngle;

	Compressor compressor;

	/*--------------------------------------------------------------
	 *						Initialization
	 * -------------------------------------------------------------
	 */

	Robot():
		left(1),right(2),drive(left,right),mainStick(0),specials(1),
		shifter(PCMA,0,1),goingUpA(PCMA,2,3),goingUpB(PCMA,4,5),
		shooter(2),shootyStick(PCMB,0,1),
		shooterAngle(PCMA,6,7),compressor(PCMA)
	{}


	void RobotInit()
	{
		autoMode = new SendableChooser();
		autoMode->AddDefault("Auto Off", (void*)&"Auto Off");
		autoMode->AddObject("Full Auto", (void*)&"Full");
		autoMode->AddObject("Breach Only (TEST)",(void*)&"Breach");
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
		toBreach->AddDefault("2",(void*)&"0");
		toBreach->AddObject("3",(void*)&"1");
		toBreach->AddObject("4",(void*)&"2");
		toBreach->AddObject("5",(void*)&"3");

		SmartDashboard::PutData("Breach", toBreach);

		for(int j=0;j<4;j++)
		{
			def[j] = new SendableChooser();

			//Add each option
			def[j]->AddDefault("Portcullis", (void*)&"Portcullis");
			def[j]->AddObject("Cheval", (void*)&"Cheval");
			def[j]->AddObject("Moat", (void*)&"Moat");
			def[j]->AddObject("Ramparts", (void*)&"Ramparts");
			def[j]->AddObject("Drawbridge", (void*)&"Drawbridge");
			def[j]->AddObject("Sally Door", (void*)&"SallyDoor");
			def[j]->AddObject("Rock Wall", (void*)&"RockWall");
			def[j]->AddObject("Rough Terrain", (void*)&"RoughTerrain");

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
		if(specials.GetRawButton(7))
		{
			goingUpA.Set(goingUpA.kForward);
			goingUpB.Set(goingUpB.kForward);
		}
		else if(specials.GetRawButton(8))
		{
			goingUpA.Set(goingUpA.kReverse);
			goingUpB.Set(goingUpB.kReverse);
		}
		else
		{
			goingUpA.Set(goingUpA.kOff);
			goingUpB.Set(goingUpB.kOff);
		}


		//Shooter
		shooter.SetSpeed(specials.GetY());
		if(specials.GetRawButton(1))
		{
			shootyStick.Set(shootyStick.kForward);
		}
		else
		{
			shootyStick.Set(shootyStick.kReverse);
		}

		//Shooter angle
		if(specials.GetRawButton(2))
		{
			shooterAngle.Set(shooterAngle.kForward);
		}
		else if(specials.GetRawButton(3))
		{
			shooterAngle.Set(shooterAngle.kReverse);
		}
		else
		{
			shooterAngle.Set(shooterAngle.kOff);
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
		drive.ArcadeDrive(0.0,0);
		shooter.SetSpeed(0);
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
		DriverStation::ReportError("Getting Auto mode...\n");
		autoSelected = *((std::string*)autoMode->GetSelected());
		DriverStation::ReportError("Mode: "+autoSelected);

		//toBreach = (SendableChooser*) SmartDashboard::GetData("Breach");
		std::string pos = *((std::string*)toBreach->GetSelected());
		if(pos=="0") breachPos=0;
		if(pos=="1") breachPos=1;
		if(pos=="2") breachPos=2;
		if(pos=="3") breachPos=3;

		for(int j=0;j<4;j++)
		{
			defense[j] = *((std::string*)def[j]->GetSelected());
			DriverStation::ReportError(defense[j]);
		}
		DriverStation::ReportError("Done!");

		ahrs->ResetDisplacement();
		ahrs->ZeroYaw();

	}

	void AutonomousPeriodic()
	{
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
		if(defense[toBreachPos]=="Portcullis"){
			BreachPortcullis();
		}
		if(defense[toBreachPos]=="Cheval"){
			BreachCheval();
		}
		if(defense[toBreachPos]=="Moat"){
			BreachMoat();
		}
		if(defense[toBreachPos]=="Ramparts"){
			BreachRamparts();
		}
		if(defense[toBreachPos]=="Drawbridge"){
			BreachDrawbridge();
		}
		if(defense[toBreachPos]=="SallyPort"){
			BreachSally();
		}
		if(defense[toBreachPos]=="RockWall"){
			BreachRockWall();
		}
		if(defense[toBreachPos]=="RoughTerrain"){
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
