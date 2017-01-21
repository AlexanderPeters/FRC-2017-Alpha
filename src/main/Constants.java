package main;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

//This is an interface that allows all constants stored here to be visible to other classes
public interface Constants {
	//public static int myMotorControllerPort = 2;
	//public static double joystickDeadband = 0.02;
	public static int myIntakeMotor = 0;
	public static int xboxPort = 0; //Xbox USB Port
	
	
	public static int intakeForwardSpeed = 9;
	
	/***************
	 * Can bus Talon id's
	 ***************/
	public static int leftDriveMasterTalon = 2;
	public static int leftDriveSlaveTalon1 = 3;
	//public static int leftDriveSlaveTalon2 = 2;
	public static int rightDriveMasterTalon = 4;
	public static int rightDriveSlaveTalon1 = 5;
	//public static int rightDriveSlaveTalon2 = 5;
	
	// PNEUMATIC STATES
	public final DoubleSolenoid.Value EXT = Value.kForward;
	public final DoubleSolenoid.Value RET = Value.kReverse;
	public final DoubleSolenoid.Value OFF = Value.kOff;
	
	/***************
	 * PDP Can Bus Port
	 ***************/
	public static int pdpPort = 0;
	
	
	
	/***************
	 * PCM Can Bus Port
	 ***************/
	public static int pcmPort = 1;
	
	// PNEUMATICS (PCM)
	public final int SHIFTER_EXT = 1;
	public final int SHIFTER_RET = 0;
	
	
	// TALON CONTROL MODES
	
	public final TalonControlMode PERCENT_VBUSMODE = TalonControlMode.PercentVbus;
	public final TalonControlMode VOLTAGE_MODE = TalonControlMode.Voltage;
	public final TalonControlMode SLAVE_MODE = TalonControlMode.Follower;
	public final TalonControlMode DISABLED = TalonControlMode.Disabled;
	// TALON BRAKE MODES
	public final boolean BRAKE_MODE = true;
	public final boolean COAST_MODE = false;
	
	
	
	
}
