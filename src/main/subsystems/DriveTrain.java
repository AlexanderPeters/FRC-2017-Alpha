package main.subsystems;

import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import Util.DriveHelper;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.TalonSRX;
import main.Constants;
import main.Robot;
import main.commands.drivetrain.Drive;


public class DriveTrain extends Subsystem implements Constants{
	public static DriveTrain instance;
	private AHRS NavX = Robot.getNavX();
	private DriveHelper helper = new DriveHelper(5);
	private static TalonSRX leftDriveMaster = new TalonSRX(Constants.LEFT_Drive_Master);
	private static TalonSRX leftDriveSlave1 = new TalonSRX(Constants.LEFT_Drive_SLAVE1);
	//private static TalonSRX leftDriveSlave2 = new TalonSRX(Constants.leftDriveSlaveTalon2);
	private static TalonSRX rightDriveMaster = new TalonSRX(Constants.RIGHT_Drive_Master);
	private static TalonSRX rightDriveSlave1 = new TalonSRX(Constants.RIGHT_Drive_SLAVE1);
	//private static TalonSRX rightDriveSlave2 = new TalonSRX(Constants.rightDriveSlaveTalon2);
	
	public DriveTrain getInstance() {
		if(instance == null) {
			instance = new DriveTrain();
		}
			return instance;
	}

	public void drive(double throttle, double heading) {
		leftDriveSlave1.set(heading);
		
		
	}
	/*******************
	 * SUPPORT METHODS *
	 *******************/
	
	/**
	 * Reverses the output of the Talon SRX's
	 * 
	 * @param output - Whether the output should be reversed.
	 */
	private void reverseTalons(boolean output) {
		leftDriveMaster.reverseOutput(output);
		rightDriveMaster.reverseOutput(output);
	}

	/**
	 * Sets the Talon SRX's brake mode
	 * 
	 * @param brake - Sets the brake mode (Uses default brake modes)
	 */
	private void setBrakeMode(Boolean brake) {
		leftDriveMaster.enableBrakeMode(brake);
		leftDriveSlave1.enableBrakeMode(brake);
		rightDriveMaster.enableBrakeMode(brake);
		rightDriveSlave1.enableBrakeMode(brake);
	}

	/**
	 * Sets the Talon SRX's control mode
	 * 
	 * @param mode - Sets the control mode (Uses default control modes)
	 */
	private void setCtrlMode(TalonControlMode mode) {
		leftDriveMaster.changeControlMode(mode);
		leftDriveSlave1.changeControlMode(SLAVE_MODE);
		leftDriveSlave1.set(leftDriveMaster.getDeviceID());
		rightDriveMaster.changeControlMode(mode);
		rightDriveSlave1.changeControlMode(SLAVE_MODE);
		rightDriveSlave1.set(rightDriveMaster.getDeviceID());
	}

	/**
	 * Sets the Talon SRX's defaults (reversing, brake and control modes)
	 */
	private void setTalonDefaults() {
		reverseTalons(true);
		setBrakeMode(true);
		setCtrlMode(DEFAULT_CTRL_MODE);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new Drive());
		
	}

}
