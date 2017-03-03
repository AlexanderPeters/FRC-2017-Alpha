package main;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import lib.joystick.XboxController;

public interface HardwareAdapter {
	//OI
	public static XboxController xbox = new XboxController(Constants.Xbox_Port);
	
	//DriveTrain
	//Verified for Competition Bot 1 2/13/17
	public static CANTalon leftDriveMaster = new CANTalon(Constants.LEFT_Drive_Master);
	public static CANTalon leftDriveSlave1 = new CANTalon(Constants.LEFT_Drive_Slave1);
	//public static CANTalon leftDriveSlave2 = new CANTalon(Constants.LEFT_Drive_Slave2);
	public static CANTalon rightDriveMaster = new CANTalon(Constants.RIGHT_Drive_Master);
	public static CANTalon rightDriveSlave1 = new CANTalon(Constants.RIGHT_Drive_Slave1);
	//public static CANTalon rightDriveSlave2 = new CANTalon(Constants.RIGHT_Drive_Slave2);
	
	//Shooter
	public static CANTalon shooter = new CANTalon(Constants.Shooter_Flywheel);
	public static Servo hoodServo = new Servo(Constants.Shooter_Hood);
	public static DigitalInput hoodSwitch = new DigitalInput(Constants.Shooter_Switch);
	public static Servo shooterIndexer = new Servo(Constants.Shooter_Indexer);
	
	//Climber
	public static Spark climberMotor = new Spark(Constants.Climber_Motor);
	//public static Servo climberLeft = new Servo(Constants.LEFT_Climber_Intake);
	//public static Servo climberRight = new Servo(Constants.RIGHT_Climber_Intake);
	
	//Intake
	public static Spark intakeMotor = new Spark(Constants.Intake_Motor);
	
	//Stirrer
	public static Spark stirrerMotor = new Spark(Constants.Stirrer_Motor);
	
	//Pnuematics
	public static DoubleSolenoid shifter = new DoubleSolenoid(Constants.PCM_Port, Constants.SHIFTER_EXT, Constants.SHIFTER_RET);
	public static DoubleSolenoid gearMech = new DoubleSolenoid(Constants.PCM_Port, Constants.GEAR_EXT, Constants.GEAR_RET);
	public static Compressor comp = new Compressor(Constants.PCM_Port);
	
	//PDP
	public static PowerDistributionPanel pdp = new PowerDistributionPanel(Constants.PDP_Port);
	

}
