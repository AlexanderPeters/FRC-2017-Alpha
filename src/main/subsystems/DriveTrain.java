package main.subsystems;

import com.kauailabs.navx.frc.AHRS;

//Example import
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
//Need for sure
import edu.wpi.first.wpilibj.command.Subsystem;
import lib.Rotation2d;
import main.Constants;
import main.OI;
import main.Robot;

	import java.util.Set;

	
	import com.team254.lib.util.Kinematics;
	import com.team254.frc2016.RobotState;
	import com.team254.frc2016.loops.Loop;
	import com.team254.lib.util.ADXRS453_Gyro;
	import com.team254.lib.util.AdaptivePurePursuitController;
	import com.team254.lib.util.DriveSignal;
	import com.team254.lib.util.Path;
	import com.team254.lib.util.RigidTransform2d;
	import com.team254.lib.util.Rotation2d;

	import com.team254.lib.util.SynchronousPID;
	import edu.wpi.first.wpilibj.CANTalon;
	import edu.wpi.first.wpilibj.Counter;
	import edu.wpi.first.wpilibj.DigitalInput;
	import edu.wpi.first.wpilibj.DriverStation;
	import edu.wpi.first.wpilibj.Solenoid;
	import edu.wpi.first.wpilibj.Timer;
	import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

	/**
	 * The robot's drivetrain, which implements the Superstructure abstract class.
	 * The drivetrain has several states and builds on the abstract class by
	 * offering additional control methods, including control by path and velocity.
	 * 
	 * @see Subsystem.java
	 */
	public class Drive extends Subsystem {
	    protected static final int kVelocityControlSlot = 0;
	    protected static final int kBaseLockControlSlot = 1;

	    private static Drive instance_ = new Drive();
	    private double mLastHeadingErrorDegrees;

	    public static Drive getInstance() {
	        return instance_;
	    }

	    // The robot drivetrain's various states
	    public enum DriveControlState {
	        OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
	    }

	    private static CANTalon leftDriveMaster = new CANTalon(Constants.leftDriveMasterTalon);
		private static CANTalon leftDriveSlave1 = new CANTalon(Constants.leftDriveSlaveTalon1);
		//private static CANTalon leftDriveSlave2 = new CANTalon(Constants.leftDriveSlaveTalon2);
		private static CANTalon rightDriveMaster = new CANTalon(Constants.rightDriveMasterTalon);
		private static CANTalon rightDriveSlave1 = new CANTalon(Constants.rightDriveSlaveTalon1);
		//private static CANTalon rightDriveSlave2 = new CANTalon(Constants.rightDriveSlaveTalon2);
		
		private AHRS NavX = Robot.getNavX();

	    private DriveControlState driveControlState_;
	    private VelocityHeadingSetpoint velocityHeadingSetpoint_;
	    private AdaptivePurePursuitController pathFollowingController_;
	    private SynchronousPID velocityHeadingPid_;

	    // The main control loop (an implementation of Loop), which cycles
	    // through different robot states
	    private final Loop mLoop = new Loop() {
	        @Override
	        public void onStart() {
	            setOpenLoop(DriveSignal.NEUTRAL);
	            pathFollowingController_ = null;
	            setBrakeMode(false);
	            
	        }

	        @Override
	        public void onLoop() {
	            synchronized (Drive.this) {
	                
	                switch (driveControlState_) {
	                case OPEN_LOOP:
	                    return;
	                case BASE_LOCKED:
	                    return;
	                case VELOCITY_SETPOINT:
	                    // Talons are updating the control loop state
	                    return;
	                case VELOCITY_HEADING_CONTROL:
	                    updateVelocityHeadingSetpoint();
	                    return;
	                case PATH_FOLLOWING_CONTROL:
	                    updatePathFollower();
	                    if (isFinishedPath()) {
	                        stop();
	                    }
	                    break;
	                default:
	                    System.out.println("Unexpected drive control state: " + driveControlState_);
	                    break;
	                }
	            }
	        }

	        @Override
	        public void onStop() {
	            setOpenLoop(DriveSignal.NEUTRAL);
	        }
	    };

	    // The constructor instantiates all of the drivetrain components when the
	    // robot powers up
	    private Drive() {
	        leftDriveMaster = new CANTalon(Constants.leftDriveMasterTalon);
	        leftDriveSlave1 = new CANTalon(Constants.leftDriveSlaveTalon1);
	        rightDriveMaster = new CANTalon(Constants.rightDriveMasterTalon);
	        rightDriveSlave1 = new CANTalon(Constants.leftDriveSlaveTalon1);
	        setBrakeMode(true);
	       
	        
	        setHighGear(true);
	        gyro_ = new ADXRS453_Gyro();
	        

	        // Get status at 100Hz
	        leftDriveMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
	        rightDriveMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

	        // Start in open loop mode
	        leftDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	        leftDriveMaster.set(0);
	        leftSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
	        leftSlave_.set(Constants.kLeftDriveMasterId);
	        rightDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	        rightDriveMaster.set(0);
	        rightSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
	        rightSlave_.set(Constants.kRightDriveMasterId);
	        setBrakeMode(false);

	        // Set up the encoders
	        leftDriveMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
	        if (leftDriveMaster.isSensorPresent(
	                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
	            DriverStation.reportError("Could not detect left drive encoder!", false);
	        }
	        leftDriveMaster.reverseSensor(true);
	        leftDriveMaster.reverseOutput(false);
	        leftSlave_.reverseOutput(false);
	        rightDriveMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
	        if (rightDriveMaster.isSensorPresent(
	                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
	            DriverStation.reportError("Could not detect right drive encoder!", false);
	        }
	        rightDriveMaster.reverseSensor(false);
	        rightDriveMaster.reverseOutput(true);
	        rightSlave_.reverseOutput(false);

	        // Load velocity control gains
	        leftDriveMaster.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
	                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
	                kVelocityControlSlot);
	        rightDriveMaster.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
	                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
	                kVelocityControlSlot);
	        // Load base lock control gains
	        leftDriveMaster.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
	                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
	                kBaseLockControlSlot);
	        rightDriveMaster.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
	                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
	                kBaseLockControlSlot);

	        velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi,
	                Constants.kDriveHeadingVelocityKd);
	        velocityHeadingPid_.setOutputRange(-30, 30);

	        setOpenLoop(DriveSignal.NEUTRAL);
	    }

	    public Loop getLoop() {
	        return mLoop;
	    }

	    protected synchronized void setLeftRightPower(double left, double right) {
	        leftDriveMaster.set(left);
	        rightDriveMaster.set(-right);
	    }

	    public synchronized void setOpenLoop(DriveSignal signal) {
	        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
	            leftDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	            rightDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	            driveControlState_ = DriveControlState.OPEN_LOOP;
	        }
	        setLeftRightPower(signal.leftMotor, signal.rightMotor);
	    }

	    public synchronized void setBaseLockOn() {
	        if (driveControlState_ != DriveControlState.BASE_LOCKED) {
	            leftDriveMaster.setProfile(kBaseLockControlSlot);
	            leftDriveMaster.changeControlMode(CANTalon.TalonControlMode.Position);
	            leftDriveMaster.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
	            leftDriveMaster.set(leftDriveMaster.getPosition());
	            rightDriveMaster.setProfile(kBaseLockControlSlot);
	            rightDriveMaster.changeControlMode(CANTalon.TalonControlMode.Position);
	            rightDriveMaster.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
	            rightDriveMaster.set(rightDriveMaster.getPosition());
	            driveControlState_ = DriveControlState.BASE_LOCKED;
	            setBrakeMode(true);
	        }
	        setHighGear(false);
	    }

	    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
	        configureTalonsForSpeedControl();
	        driveControlState_ = DriveControlState.VELOCITY_SETPOINT;
	        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
	    }

	    public synchronized void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) {
	        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL) {
	            configureTalonsForSpeedControl();
	            driveControlState_ = DriveControlState.VELOCITY_HEADING_CONTROL;
	            velocityHeadingPid_.reset();
	        }
	        velocityHeadingSetpoint_ = new VelocityHeadingSetpoint(forward_inches_per_sec, forward_inches_per_sec,
	                headingSetpoint);
	        updateVelocityHeadingSetpoint();
	    }

	    /**
	     * The robot follows a set path, which is defined by Waypoint objects.
	     * 
	     * @param Path
	     *            to follow
	     * @param reversed
	     * @see com.team254.lib.util/Path.java
	     */
	    public synchronized void followPath(Path path, boolean reversed) {
	        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
	            configureTalonsForSpeedControl();
	            driveControlState_ = DriveControlState.PATH_FOLLOWING_CONTROL;
	            velocityHeadingPid_.reset();
	        }
	        pathFollowingController_ = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
	                Constants.kPathFollowingMaxAccel, Constants.kLooperDt, path, reversed, 0.25);
	        updatePathFollower();
	    }

	    /**
	     * @return Returns if the robot mode is Path Following Control and the set
	     *         path is complete.
	     */
	    public synchronized boolean isFinishedPath() {
	        return (driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL && pathFollowingController_.isDone())
	                || driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL;
	    }

	    /**
	     * Path Markers are an optional functionality that name the various
	     * Waypoints in a Path with a String. This can make defining set locations
	     * much easier.
	     * 
	     * @return Set of Strings with Path Markers that the robot has crossed.
	     */
	    public synchronized Set<String> getPathMarkersCrossed() {
	        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
	            return null;
	        } else {
	            return pathFollowingController_.getMarkersCrossed();
	        }
	    }

	    public double getLeftDistanceInches() {
	        return rotationsToInches(leftDriveMaster.getPosition());
	    }

	    public double getRightDistanceInches() {
	        return rotationsToInches(rightDriveMaster.getPosition());
	    }

	    public double getLeftVelocityInchesPerSec() {
	        return rpmToInchesPerSecond(leftDriveMaster.getSpeed());
	    }

	    public double getRightVelocityInchesPerSec() {
	        return rpmToInchesPerSecond(rightDriveMaster.getSpeed());
	    }

	    

	    public synchronized Rotation2d getGyroAngle() {
	        return Rotation2d.fromDegrees(NavX.getAngle());
	    }

	    
	    public void setHighGear(boolean high_gear) {
	    	if(high_gear){
	    		new main.commands.pnuematics.ShiftUp();
	    	}
	    	else
	    		new main.commands.pnuematics.ShiftDown();
	    }

	    public synchronized void resetEncoders() {
	        leftDriveMaster.setPosition(0);
	        rightDriveMaster.setPosition(0);

	        leftDriveMaster.setEncPosition(0);
	        rightDriveMaster.setEncPosition(0);
	    }

	    public synchronized DriveControlState getControlState() {
	        return driveControlState_;
	    }

	    public synchronized VelocityHeadingSetpoint getVelocityHeadingSetpoint() {
	        return velocityHeadingSetpoint_;
	    }

	    public synchronized void stop() {
	        setOpenLoop(DriveSignal.NEUTRAL);
	    }

	    

	    public synchronized void zeroSensors() {
	        resetEncoders();
	        NavX.reset();
	    }

	    private void configureTalonsForSpeedControl() {
	        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL
	                && driveControlState_ != DriveControlState.VELOCITY_SETPOINT
	                && driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
	            leftDriveMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
	            leftDriveMaster.setProfile(kVelocityControlSlot);
	            leftDriveMaster.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
	            rightDriveMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
	            rightDriveMaster.setProfile(kVelocityControlSlot);
	            rightDriveMaster.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
	            setHighGear(true);
	            setBrakeMode(true);
	        }
	    }

	    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
	        if (driveControlState_ == DriveControlState.VELOCITY_HEADING_CONTROL
	                || driveControlState_ == DriveControlState.VELOCITY_SETPOINT
	                || driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL) {
	            leftDriveMaster.set(inchesPerSecondToRpm(left_inches_per_sec));
	            rightDriveMaster.set(inchesPerSecondToRpm(right_inches_per_sec));
	        } else {
	            System.out.println("Hit a bad velocity control state");
	            leftDriveMaster.set(0);
	            rightDriveMaster.set(0);
	        }
	    }

	    private void updateVelocityHeadingSetpoint() {
	        Rotation2d actualGyroAngle = getGyroAngle();

	        mLastHeadingErrorDegrees = velocityHeadingSetpoint_.getHeading().rotateBy(actualGyroAngle.inverse())
	                .getDegrees();

	        double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
	        updateVelocitySetpoint(velocityHeadingSetpoint_.getLeftSpeed() + deltaSpeed / 2,
	                velocityHeadingSetpoint_.getRightSpeed() - deltaSpeed / 2);
	    }

	    private void updatePathFollower() {
	        RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
	        RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
	        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

	        // Scale the command to respect the max velocity limits
	        double max_vel = 0.0;
	        max_vel = Math.max(max_vel, Math.abs(setpoint.left));
	        max_vel = Math.max(max_vel, Math.abs(setpoint.right));
	        if (max_vel > Constants.kPathFollowingMaxVel) {
	            double scaling = Constants.kPathFollowingMaxVel / max_vel;
	            setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
	        }
	        updateVelocitySetpoint(setpoint.left, setpoint.right);
	    }

	    

	    private static double rotationsToInches(double rotations) {
	        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
	    }

	    private static double rpmToInchesPerSecond(double rpm) {
	        return rotationsToInches(rpm) / 60;
	    }

	    private static double inchesToRotations(double inches) {
	        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	    }

	    private static double inchesPerSecondToRpm(double inches_per_second) {
	        return inchesToRotations(inches_per_second) * 60;
	    }

	    public void setBrakeMode(boolean on) {
	        
	            leftDriveMaster.enableBrakeMode(on);
	            leftDriveSlave1.enableBrakeMode(on);
	            rightDriveMaster.enableBrakeMode(on);
	            rightDriveSlave1.enableBrakeMode(on);
	          
	    }
	    

	    /**
	     * VelocityHeadingSetpoints are used to calculate the robot's path given the
	     * speed of the robot in each wheel and the polar coordinates. Especially
	     * useful if the robot is negotiating a turn and to forecast the robot's
	     * location.
	     */
	    public static class VelocityHeadingSetpoint {
	        private final double leftSpeed_;
	        private final double rightSpeed_;
	        private final Rotation2d headingSetpoint_;

	        // Constructor for straight line motion
	        public VelocityHeadingSetpoint(double leftSpeed, double rightSpeed, Rotation2d headingSetpoint) {
	            leftSpeed_ = leftSpeed;
	            rightSpeed_ = rightSpeed;
	            headingSetpoint_ = headingSetpoint;
	        }

	        public double getLeftSpeed() {
	            return leftSpeed_;
	        }

	        public double getRightSpeed() {
	            return rightSpeed_;
	        }

	        public Rotation2d getHeading() {
	            return headingSetpoint_;
	        }
	    }

		@Override
		protected void initDefaultCommand() {
			// TODO Auto-generated method stub
			
		}
	}
