#include "WPILib.h"
#include "AHRS.h"

class Robot: public IterativeRobot
{
public:
	LiveWindow *lw = LiveWindow::GetInstance();

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
	//AHRS *ahrs;


	//Human input
	Joystick mainStick;
	Joystick specials;


	//Special things
	Compressor compressor;
	DoubleSolenoid shifter;
	DoubleSolenoid goingUpA, goingUpB;
	Victor shooter;
	DoubleSolenoid shootyStick;

	/*--------------------------------------------------------------
	 *						Initialization
	 * -------------------------------------------------------------
	 */
	Robot():
		left(0),right(1),drive(left,right),
		mainStick(0), specials(1), compressor(0), shifter(0,1),
		goingUpA(2,3),goingUpB(4,5),shooter(2),shootyStick(6,7)
	{}

	void RobotInit()
	{
		autoMode = new SendableChooser();
		autoMode->AddDefault("Auto Off", (void*)&"Auto Off");
		autoMode->AddObject("Breach Defense", (void*)&"Breach");
		SmartDashboard::PutData("Auto Modes", autoMode);


		try {
			/* Communicate w/navX MXP via the MXP SPI Bus.                                       */
			/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
			//ahrs = new AHRS(SPI::Port::kMXP);
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

			SmartDashboard::PutData("Defense "+j, def[j]);
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
	 * 							Autonomous		TODO
	 * ----------------------------------------------------------------------
	 */
	void AutonomousInit()
	{
		autoState = 0;
		drive.ArcadeDrive(0.0,0);
		DriverStation::ReportError("Test");
		drive.SetSafetyEnabled(false);
		left.SetSafetyEnabled(false);
		right.SetSafetyEnabled(false);
		//autoMode = (SendableChooser*) SmartDashboard::GetData("Auto Modes");
		autoSelected = *((std::string*)autoMode->GetSelected());
		DriverStation::ReportError(autoSelected);

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
	}

	void AutonomousPeriodic()
	{
		if(autoSelected=="Breach")
		{

			switch(autoState)
			{
			case 0: {
				//TODO Use move function to move forward ? meters
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
			}
		}
	}





	/*------------------------------------------------------------------------
	 *                        BREACHING        TODO
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
 * 							NavX-MXP Navigation    TODO
 * ---------------------------------------------------------------------
 */









/* ----------------------------------------------------------------------
 * 							Image Processing     TODO
 * ----------------------------------------------------------------------
 */
};

START_ROBOT_CLASS(Robot)
