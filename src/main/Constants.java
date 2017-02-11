package main;

import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


//This is an interface that allows all constants stored here to be visible to other classes
public interface Constants {
	/*************
	 * VARIABLES *
	 *************/
	// THROTTLE MULTIPLIERS
	public final double intakeMotorForward = -1.0;
	public final double intakeMotorReverse = 1.0;
	public final double intakeMotorOff = 0.0;
	public final double climberMotorForwardFast = 1;
	public final double climberMotorForwardSlow = 0.25; //Needs to be field tested so that the climber doesn't stop turning
	public final int shooterMotorForward = 1;
	public final double stirrerMotorOn = 1.0;
	public final double stirrerMotorReverse = -1.0;
	public final double stirrerMotorOff = 0.0;
	public final double driveThrottle = 1.0;
	public final double turnThrottle = 1.0;
	
	// JOYSTICK DEADBAND'S
	public final double throttleDeadband = 0.02;
	public final double headingDeadband = 0.02;
	
	// PID VALUES FOR AUTONOMOUS
	public final double rightWheelVelocityKP = 0.0;
	public final double rightWheelPositionKP = 0.0;
	public final double leftWheelVelocityKP = 0.0;
	public final double leftWheelPositionKP = 0.0;
	public final double headingControllerKP = 0.0;
	
	//PID VALUES FOR DRIVETRAIN
	public final double turnInPlaceKP = 0.03;
	public final double turnInPlaceKI = 0.0;
	public final double turnInPlaceKD = 0.0;
	public final double turnInPlaceKF = 0.0;
	public final double kToleranceDegrees = 2.0f;//Subject to change
	public final double leftDisplacementKP = 0.0;
	public final double leftDisplacementKI = 0.0;
	public final double leftDisplacementKD = 0.0;
	public final int leftDisplacementTolerance = 0;
	public final double rightDisplacementKP = 0.0;
	public final double rightDisplacementKI = 0.0;
	public final double rightDisplacementKD = 0.0;
	public final int rightDisplacementTolerance = 0;
	
	/*************
	 * CONSTANTS *
	 *************/
	//DOH
	public final String DOH = "DOH!";
	
	//Loop Time
	public final double kLooperDt = 0.01;
	
	// DEFAULT TALON MODES
	public final TalonControlMode DEFAULT_CTRL_MODE = TalonControlMode.PercentVbus;//TalonControlMode.PercentVbus;
	public final boolean DEFAULT_BRAKE_MODE = true;
	// TALON CONTROL MODES
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
	//ENCODER CODES PER REV
	public final int codesPerRev = 0;
	//UDP_PORT
	public final int udpPort = 5800;
	//DRIVERCAM_FPS
	public final int fps = 30;
	
	/****************
	 * DEVICE PORTS *
	 ****************/
	// JOYSTICKS (USB)
	public final int Xbox_Port = 0;
	// TALON SRX'S (CAN BUS)
	public final int LEFT_Drive_Master = 2;
	public final int LEFT_Drive_Slave1 = 3;
	public final int LEFT_Drive_Slave2 = 4;
	public final int RIGHT_Drive_Master = 5;
	public final int RIGHT_Drive_Slave1 = 6;
	public final int RIGHT_Drive_Slave2 = 7;
	// OTHER MOTOR CONTROLLERS (PWM)
	public final int Intake_Motor = 0;
	public final int Climber_Motor = 1;
	public final int Stirrer_Motor = 2;
	// PNEUMATICS (PCM)
	public final int GEAR_EXT = 3;
	public final int GEAR_RET = 2;
	public final int SHIFTER_EXT = 1;
	public final int SHIFTER_RET = 0;
	//CAN BUS (Other Devices)
	public final int PDP_Port = 0;
	public final int PCM_Port = 1;
	
		
}
