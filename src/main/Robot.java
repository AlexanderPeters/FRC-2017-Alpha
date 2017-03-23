package main;

import Util.SmartDashboardInteractions;
import controllers.UDPController;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.Looper;
import lib.UDPForVision;
import main.commands.auto.centerGearAuto;
import main.commands.auto.doNothing;
import main.commands.auto.leftGearAuto;
import main.commands.auto.rightGearAuto;
import main.commands.auto.shootingAuto;
import main.subsystems.Climber;
import main.subsystems.DriveCamera;
import main.subsystems.DriveTrain;
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
	public static enum GameState {
		Initializing, Test, Teleop, Autonomous
	}
	public static enum RobotState {
		Driving, Climbing, Neither
	}

	public static OI oi;
	public static DriveTrain dt;
	public static Pneumatics pn;
	public static Climber cl;
	public static Intake in;
	public static Stirrer str;
	public static FlyWheel shooter;
	public static Hood hd;
	public static Indexer id;
	public static DriveCamera dc;
	public static SmartDashboardInteractions sdb;
	public static GameState gameState;
	public static RobotState robotState = RobotState.Neither;
	//    public static Looper mEnabledLooper = new Looper(kEnabledLooperDt);
    public static Looper mLooper = new Looper(kLooperDt);
    public static UDPForVision comms = new UDPForVision();
	
    Command autoCommand;
    SendableChooser<Command> chooser;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @SuppressWarnings("deprecation")
    public void robotInit() {

		
    	gameState = GameState.Initializing;
		pn = new Pneumatics();	
		str = new Stirrer();
		dt = new DriveTrain();
		cl = new Climber();
		in = new Intake();
		id = new Indexer();
		shooter = new FlyWheel();
		hd = new Hood();
		dc = new DriveCamera();
		//This has to be last as the subsystems can not be null when a command requires them
		oi = new OI();

		//mEnabledLooper.register(new UDPController());
		mLooper.register(new UDPController());
		mLooper.start();
    
		chooser = new SendableChooser<Command>();
        chooser.addDefault("Do Nothing Auto", new doNothing());
        chooser.addObject("Shooting Auto", new shootingAuto());
        chooser.addObject("Center Gear Auto", new centerGearAuto());
        SmartDashboard.putData("Auto mode", chooser);
        
        SmartDashboard.putDouble("Turning KP Big Angle", turnInPlaceKPBigAngle);
        SmartDashboard.putDouble("Turning KI Big Angle", turnInPlaceKIBigAngle);
        SmartDashboard.putDouble("Turning KD Big Angle", turnInPlaceKDBigAngle);
        SmartDashboard.putDouble("Turning MinVoltage Big Angle", kMinVoltageTurnBigAngle);
        SmartDashboard.putDouble("Turning MaxVoltage Big Angle", kMaxVoltageTurnBigAngle);
        
        SmartDashboard.putDouble("Turning KP Small Angle", turnInPlaceKPSmallAngle);
        SmartDashboard.putDouble("Turning KI Small Angle", turnInPlaceKISmallAngle);
        SmartDashboard.putDouble("Turning KD Small Angle", turnInPlaceKDSmallAngle);
        SmartDashboard.putDouble("Turning MinVoltage Small Angle", kMinVoltageTurnSmallAngle);
        SmartDashboard.putDouble("Turning MaxVoltage Small Angle", kMaxVoltageTurnSmallAngle);

        SmartDashboard.putDouble("Turning Tolerance", kToleranceDegreesDefault);
        SmartDashboard.putInt("Turn In Place Controller Switch Angle", turnInPlaceControllerSwitchAngle);
        
        SmartDashboard.putDouble("Distance KP", displacementKP);
        SmartDashboard.putDouble("Distance KI", displacementKI);
        SmartDashboard.putDouble("Distance KD", displacementKD);
        SmartDashboard.putDouble("Distance Tolerance", kToleranceDisplacementDefault);
        SmartDashboard.putDouble("Distance MaxVoltage", kMaxVoltageDisp);
        
        displayVisionStatus();


    }
    @SuppressWarnings("deprecation")
	private void displayVisionStatus() {
    	double range = comms.getRange();
        double distanceToGoal = (comms.getRange()*Math.cos(cameraAngle * Math.PI/180)) - desiredDistanceToGoal;
        double bearing = comms.getBearing();
        boolean targetFound = comms.getTargetFound();
        
        SmartDashboard.putDouble("Vision Bearing Estimate", bearing);
        SmartDashboard.putDouble("Vision Range Estimate", range);
        SmartDashboard.putBoolean("Target Found", targetFound);
        
        
        if(targetFound && bearing <= bearingToGoalTolerance && bearing >= -bearingToGoalTolerance)
        	SmartDashboard.putBoolean("Bearing To Target Aligned", true);
        else
        	SmartDashboard.putBoolean("Bearing To Target Aligned", false);
        
        if(targetFound && distanceToGoal <= distToGoalTolerance && distanceToGoal >= -distToGoalTolerance)
        	SmartDashboard.putBoolean("Distance To Target Aligned", true);
        else
        	SmartDashboard.putBoolean("Distance To Target Aligned", false);
        
        if(targetFound && distanceToGoal <= distToGoalTolerance && distanceToGoal >= -distToGoalTolerance && 
        		bearing <= bearingToGoalTolerance && bearing >= -bearingToGoalTolerance)
        	SmartDashboard.putBoolean("Target In Range", true);
        else
        	SmartDashboard.putBoolean("Target In Range", false);
        
        SmartDashboard.putBoolean("Ready To Start Match", comms.connectedAtStartOfMatch());
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){
		// Configure loopers
        //mEnabledLooper.stop();
    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}
	
	/**
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
    	autoCommand = (Command) chooser.getSelected();
    	
    	// Configure loopers
        //mEnabledLooper.start();
    	
    	if(autoCommand != null) autoCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	gameState = GameState.Autonomous;
        displayVisionStatus();
        Scheduler.getInstance().run();
    }
    
    public void teleopInit() {
    	gameState = GameState.Teleop;
    	
    	// Configure loopers
        //mEnabledLooper.start();
    	
    	/* This makes sure that the autonomous stops running when
           teleop starts running. If you want the autonomous to 
           continue until interrupted by another command, remove
           this line or comment it out. */
    	
        if (autoCommand != null) autoCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	gameState = GameState.Teleop;
        displayVisionStatus();
        //System.out.println("GameState " + gameState + "Robot State " + robotState);
    	Scheduler.getInstance().run();
    }
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
   
}
