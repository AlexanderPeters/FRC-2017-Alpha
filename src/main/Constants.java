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
		
	
	/*************
	 * CONSTANTS *
	 *************/
	//DOH
	public final String doh = "DOH!";
	// DEFAULT TALON MODES
	public final TalonControlMode DEFAULT_CTRL_MODE = TalonControlMode.PercentVbus;
	public final boolean DEFAULT_BRAKE_MODE = true;
	// TALON CONTROL MODES
	public final TalonControlMode PERCENT_VBUS_MODE = TalonControlMode.PercentVbus;
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
	
	/****************
	 * DEVICE PORTS *
	 ****************/
	// JOYSTICKS (USB)
	public final int Xbox_Port = 0;
	// TALON SRX'S (CAN BUS)
	public final int LEFT_Drive_Master = 2;
	public final int LEFT_Drive_SLAVE1 = 3;
	public final int RIGHT_Drive_Master = 4;
	public final int RIGHT_Drive_SLAVE1 = 5;
	// OTHER MOTOR CONTROLLERS (PWM)
	public final int Intake_Motor = 0;
	// PNEUMATICS (PCM)
	public final int SHIFTER_EXT = 1;
	public final int SHIFTER_RET = 0;
	//CAN BUS (Other Devices)
	public final int PDP_Port = 0;
	public final int PCM_Port = 1;
		
}
