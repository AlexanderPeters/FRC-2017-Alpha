package main;

import Util.SmartDashboardInteractions;
//import controllers.TeleopCameraController;
import controllers.TrajectoryDriveController;
import controllers.UDPController;
//import edu.wpi.first.wpilibj.DriverStation;
//Necessary wpilib imports
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.vision.CameraServer;
import lib.Looper;
import lib.UDPForVision;
import main.commands.auto.centerGearAuto;
//import main.commands.stirrer.Stir;
//import main.subsystems.CameraController;
//import main.subsystems.CameraController;
//import main.subsystems.FlyWheel;
import main.subsystems.Climber;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Subsystem imports
import main.subsystems.DriveTrain;
//import main.subsystems.DriverCamera;
import main.subsystems.FlyWheel;
import main.subsystems.Hood;
import main.subsystems.Indexer;
import main.subsystems.Intake;
import main.subsystems.Pneumatics;
import main.subsystems.Stirrer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements Constants{

	//public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	//private boolean lowGear = true;
	public static enum GameState {
		Initializing, Test, Teleop, Autonomous
	}
	public static enum RobotState {
		Driving, Climbing, Neither
	}

	//private static final double kAutoLooperDt = 0;
	
	public static OI oi;
	public static DriveTrain dt;
	public static Pneumatics pn;
	public static Climber cl;
	public static Intake in;
	public static Stirrer str;
	public static FlyWheel shooter;
	public static Hood hd;
	public static Indexer id;
	public static SmartDashboardInteractions sdb;
	public static GameState gameState;
	public static RobotState robotState = RobotState.Neither;
	public static CameraServer cam;
	//private static double kEnabledLooperDt;
	
	// Enabled looper is called at 100Hz whenever the robot is enabled
    public static Looper mEnabledLooper = new Looper(kEnabledLooperDt);
    // Disabled looper is called at 100Hz whenever the robot is disabled
    //public static Looper mDisabledLooper = new Looper();
    public static Looper mAutonomousLooper = new Looper(kAutoLooperDt);
    
    public static UDPForVision comms = new UDPForVision();
	//public static DriverCamera dc = new DriverCamera(50);
	//private Thread captureThread;


	
    Command autoCommand;
   // SendableChooser chooser;
	

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		//cam.getInstance().startAutomaticCapture();
		
    	gameState = GameState.Initializing;
		pn = new Pneumatics();	
		str = new Stirrer();
		dt = new DriveTrain();
		cl = new Climber();
		in = new Intake();
		id = new Indexer();
		shooter = new FlyWheel();
		hd = new Hood();
		//dc = new DriverCamera(50);
		//This has to be last as the subsystems can not be null when a command requires them
		oi = new OI();


		/*
		UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);

		MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);

		mjpegServer1.setSource(usbCamera);*/
		// Configure loopers
        mAutonomousLooper.register(new TrajectoryDriveController());
        mEnabledLooper.register(new UDPController());
       // mEnabledLooper.register(new TeleopCameraController());
        mEnabledLooper.start();
        /*captureThread = new Thread(() -> {
  	      while (true) {
  	        dc.poke();
  	        Timer.delay(1 / (double) Constants.fps);
  	      }
  	    });*/
  	    //captureThread.setName("Camera Capture Thread");
  	    //captureThread.start();
  	  
        
		//System.out.println("init2");
		//chooser = new SendableChooser();
        //chooser.addDefault("Default Auto", new ExampleCommand());
		//chooser.addObject("My Auto", new MyAutoCommand());
        //SmartDashboard.putData("Auto mode", chooser);
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){
		//new Stir(Constants.stirrerMotorOff);


    	// Configure loopers
        //mEnabledLooper.stop();
        //mDisabledLooper.start();
    }
	
	public void disabledPeriodic() {
		//new Stir(Constants.stirrerMotorOff);

		Scheduler.getInstance().run();
	}
	

	/**4
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
    	gameState = GameState.Autonomous;
    	//mAutonomousLooper.start();
    	autoCommand = new centerGearAuto();
    	
    	if(autoCommand != null)
    		autoCommand.start();
    	//new Stir(Constants.stirrerMotorOn);

    	
    	// Configure loopers
        //mDisabledLooper.stop();
        //mEnabledLooper.start();
        
        //autonomousCommand = (Command) chooser.getSelected();
        
		/* String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
		case "My Auto":
			autonomousCommand = new MyAutoCommand();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		} */
    	
    	// schedule the autonomous command (example)
        //if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	gameState = GameState.Autonomous;
    	//new Stir(Constants.stirrerMotorOn);
        Scheduler.getInstance().run();
    }
    
    public void teleopInit() {
    	gameState = GameState.Teleop;
    	//new Stir(Constants.stirrerMotorOn);

    	// Configure loopers
        //mDisabledLooper.stop();
        mAutonomousLooper.stop();
        
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	gameState = GameState.Teleop;
    	//new Stir(Constants.stirrerMotorOn);
    	Scheduler.getInstance().run();
    }
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {

        LiveWindow.run();
    }
        
}
