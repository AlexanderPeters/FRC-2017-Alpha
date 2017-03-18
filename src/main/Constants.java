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
	public final double stirrerMotorOn = 1.0;
	public final double stirrerMotorReverse = -1.0;
	public final double stirrerMotorOff = 0.0;
	public final double driveThrottle = 1.0;
	public final double turnThrottle = 1.0;
	public final double shooterForward = 0.40;
	public final double shooterOff = 0.0;
	
	// JOYSTICK DEADBAND'S
	public final double throttleDeadband = 0.02;
	public final double headingDeadband = 0.02;
	
	//DRIVETRAIN STRAIGHT LINE kp
	public final double straightLineKP = -0.03;
	public final double straightLineKPReverse = 0.03;
	
	// PID VALUES FOR AUTONOMOUS
	public final double rightWheelVelocityKP = 0.0;
	public final double rightWheelPositionKP = 0.0;
	public final double leftWheelVelocityKP = 0.0;
	public final double leftWheelPositionKP = 0.0;
	public final double headingControllerKP = 0.0;
	
	//PID VALUES FOR DRIVETRAIN
	public final double turnInPlaceKP = 0.02;//Need to tune//0.03
	public final double turnInPlaceKI = 0.0;
	public final double turnInPlaceKD = 0.005;//0.4
	public final double turnInPlaceKF = 0.0;
	public final double kToleranceDegreesDefault = 1.0f;//Subject to change
	
	public final double displacementKP = 0.01;//Need to tune (turned way the heck down for testing tommorrow 2/25/17)
	public final double displacementKI = 0.0;
	public final double displacementKD = 0.0;
	public final double kToleranceDisplacementDefault = 0.084;//Subject to change #DAMN STRAIGHT!!!!!
	
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
