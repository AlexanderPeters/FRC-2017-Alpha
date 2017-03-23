package main;

import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


//This is an interface that allows all constants stored here to be visible to other classes
public interface Constants {
	/*************
	 * VARIABLES *
	 *************/
	// ROBOT VARIABLES
	public final boolean isCompRobot = true;//same as practice bot
	// THROTTLE MULTIPLIERS
	public final double intakeMotorForward = 1.0;
	public final double intakeMotorReverse = -1.0;
	public final double intakeMotorOff = 0.0;
	public final double climberMotorForwardFast = 1;
	public final double climberMotorForwardSlow = 0.8;
	public final double stirrerMotorForward = -1.0;
	public final double stirrerMotorReverse = 1.0;
	public final double stirrerMotorOff = 0.0;
	public final double driveThrottle = 1.0;
	public final double turnThrottle = 1.0;
	public final double shooterForward = -0.8;
	public final double shooterReverse = 0.8;
	public final double shooterOff = 0.0;
	
	// JOYSTICK DEADBAND'S
	public final double throttleDeadband = 0.02;
	public final double headingDeadband = 0.02;
	
	//DRIVETRAIN STRAIGHT LINE kp
	public final double straightLineKP = 0.03; //-
	public final double straightLineKPReverse = -0.03; //+
	
	// PID VALUES FOR AUTONOMOUS
	public final double rightWheelVelocityKP = 0.0;
	public final double rightWheelPositionKP = 0.0;
	public final double leftWheelVelocityKP = 0.0;
	public final double leftWheelPositionKP = 0.0;
	public final double headingControllerKP = 0.0;
	
	//PID VALUES FOR DRIVETRAIN
	public final double turnInPlaceKPBigAngle = 0.65;
	public final double turnInPlaceKIBigAngle = 0.0;
	public final double turnInPlaceKDBigAngle = 0.0;
	public final double kMaxVoltageTurnBigAngle = 10.5;
	public final double kMinVoltageTurnBigAngle = 5.5;
	
	public final double turnInPlaceKPSmallAngle = 0.01;
	public final double turnInPlaceKISmallAngle = 0.0;
	public final double turnInPlaceKDSmallAngle = 0.0007;
	public final double kMaxVoltageTurnSmallAngle = 9.0;
	public final double kMinVoltageTurnSmallAngle = 5.5;
	
	public final double kToleranceDegreesDefault = 1.0f;
	public final int turnInPlaceControllerSwitchAngle = 42;
	
	public final double displacementKP = 1.5;//Need to tune (turned way the heck down for testing tommorrow 2/25/17)
	public final double displacementKI = 0.1;
	public final double displacementKD = 0.0;
	public final double kToleranceDisplacementDefault = 0.042;//Subject to change #DAMN STRAIGHT!!!!!
	public final double kMaxVoltageDisp = 7.0;
	
	public final double distanceBetweenRobotAndGearPeg = (double) 1/6;//2 inches in feet
		
	//PID VALUES FOR FLYWHEEL
	public final double flyWheelKP = 0.0;
	public final double flyWheelKI = 0.0;
	public final double flyWheelKD = 0.0;
	public final int flyWheelTargetVel = 3000;
	public final int flyWheelAllowableError = 10;
	
	/*************
	 * CONSTANTS *
	 *************/
	//DOH
	public final String DOH = "DOH!";
	
	//Loop Time
	public final double kEnabledLooperDt = 0.01;
	public final double kLooperDt = 0.01;
	public final double kAutoLooperDt = 0.1;
	
	// DEFAULT TALON MODES
	public final TalonControlMode DEFAULT_CTRL_MODE = TalonControlMode.PercentVbus;
	public final boolean DEFAULT_BRAKE_MODE = true;
	// TALON CONTROL MODES
	public final TalonControlMode VELOCITY = TalonControlMode.Speed;
	public final TalonControlMode PERCENT_VBUS_MODE = TalonControlMode.PercentVbus;
	public final TalonControlMode POSITION = TalonControlMode.Position;
	public final TalonControlMode VOLTAGE_MODE = TalonControlMode.Voltage;
	public final TalonControlMode SLAVE_MODE = TalonControlMode.Follower;
	public final TalonControlMode DISABLED = TalonControlMode.Disabled;
	// TALON BRAKE MODES
	public final boolean BRAKE_MODE = true;
	public final boolean COAST_MODE = false;
	// PNEUMATIC STATES
	public final DoubleSolenoid.Value EXT = Value.kForward;
	public final DoubleSolenoid.Value RET = Value.kReverse;
	public final DoubleSolenoid.Value OFF = Value.kOff;
	//UDP_PORT
	public final int udpPort = 5803;
	//DRIVERCAM_FPS
	public final int fps = 30;
	//WHEEL_SIZE
	public final double wheelSize = 0.5;//Feet
	//Encoder velocity to wheel velocity multiplier 
	public final double wheelEncoderMult = 5.1;//low gear
	public final double wheelEncoderMultHigh = 13.5;//high gear
	//ENCODER CODES PER REV
	public final int codesPerRev = 256;//5600;
	public final double conversionFactor = 256*4*wheelEncoderMult;
	//Vision
	public final double desiredDistanceToGoal = 6.75;//ft
	public final double distToGoalTolerance = 0.25;//ft
	public final double bearingToGoalTolerance = 2;
	public final double cameraHeightAboveGround = 23/12;//ft
	public final int cameraAngle = 45;
	
	
	
	/****************
	 * DEVICE PORTS *
	 ****************/
	// JOYSTICKS (USB)
	public final int Xbox_Port = 0;
	// DIGITAL IO
	public final int Shooter_Switch = 0;
	// TALON SRX'S (CAN BUS)
	public final int LEFT_Drive_Master = 2;
	public final int LEFT_Drive_Slave1 = 3;
	//public final int LEFT_Drive_Slave2 = 4;
	public final int RIGHT_Drive_Master = 5;
	public final int RIGHT_Drive_Slave1 = 6;
	//public final int RIGHT_Drive_Slave2 = 7;
	public final int Shooter_Flywheel = 8;
	// OTHER MOTOR CONTROLLERS (PWM)
	public final int Intake_Motor = 0;
	public final int Climber_Motor = 2;
	public final int Stirrer_Motor = 1;
	public final int Shooter_Hood = 3;
	//public final int LEFT_Climber_Intake = 4;
	//public final int RIGHT_Climber_Intake = 5;
	public final int Shooter_Indexer = 6;
	// PNEUMATICS (PCM)
	public final int GEAR_EXT = 2;//Currently in by default
	public final int GEAR_RET = 5;
	public final int SHIFTER_EXT = (isCompRobot? 6:3);
	public final int SHIFTER_RET = (isCompRobot? 3:6);
	//CAN BUS (Other Devices)
	public final int PDP_Port = 0;
	public final int PCM_Port = 1;
	
		
}
