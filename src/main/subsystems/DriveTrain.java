package main.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import Util.DriveHelper;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import main.Constants;
//import main.Robot;
import main.commands.drivetrain.Drive;
//import Util.MathHelper;
//NavX import
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;



public class DriveTrain extends Subsystem implements Constants{
	private static boolean highGearState = false;
	private AHRS NavX;
	private DriveHelper helper = new DriveHelper(5);
	private boolean hasBeenDrivingStraight;
	private String currentCommand;
	private static CANTalon leftDriveMaster = new CANTalon(Constants.LEFT_Drive_Master);
	private static CANTalon leftDriveSlave1 = new CANTalon(Constants.LEFT_Drive_SLAVE1);
	private static CANTalon leftDriveSlave2 = new CANTalon(Constants.LEFT_Drive_Slave2);
	private static CANTalon rightDriveMaster = new CANTalon(Constants.RIGHT_Drive_Master);
	private static CANTalon rightDriveSlave1 = new CANTalon(Constants.RIGHT_Drive_SLAVE1);
	private static CANTalon rightDriveSlave2 = new CANTalon(Constants.RIGHT_Drive_Slave2);
	private static RobotDrive driveTrain = new RobotDrive(leftDriveMaster, rightDriveMaster);
	
	public DriveTrain() {
		setTalonDefaults();
		try {
	          /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
	          /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
	          /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
	          NavX = new AHRS(SPI.Port.kMXP); 
	      } catch (RuntimeException ex ) {
	          DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
	      }
		
		
		
		
	}
	
	public void drive(double throttle, double heading) {
		currentCommand = "drive";//Prevents 2 commands from accessing the driveTrain at the same time
		if(currentCommand == "drive"){
			hasBeenDrivingStraight = false;
			driveTrain.arcadeDrive(helper.calculateThrottle(throttle), helper.calculateTurn(heading, highGearState));
		}
	}
	public void driveStraight(double throttle){
		currentCommand = "driveStraight";//Prevents 2 commands from accessing the driveTrain at the same time
		if(currentCommand == "driveStraight"){
			if(!hasBeenDrivingStraight)
				resetGyro();
			
			hasBeenDrivingStraight = true;
			
			double theta = NavX.getAngle();
			driveTrain.arcadeDrive(helper.calculateThrottle(throttle), helper.handleOverPower(theta * -0.03)); //Make this PID Controlled
		}
	}
	//public void driveToHeading
	public void changeGearing(){
		highGearState = !highGearState;
	}
	public AHRS getGyro(){
		return NavX;
	}
	
	/*******************
	 * SUPPORT METHODS *
	 *******************/
	
	/**
	 * Reverses the output of the Talon SRX's
	 * 
	 * @param output - Whether the output should be reversed.
	 */
	private void reverseTalons(boolean output) {//Actually Works ?
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
		leftDriveSlave2.enableBrakeMode(brake);
		rightDriveMaster.enableBrakeMode(brake);
		rightDriveSlave1.enableBrakeMode(brake);
		rightDriveSlave2.enableBrakeMode(brake);
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
		leftDriveSlave2.changeControlMode(SLAVE_MODE);
		leftDriveSlave2.set(leftDriveMaster.getDeviceID());
		
		rightDriveMaster.changeControlMode(mode);
		rightDriveSlave1.changeControlMode(SLAVE_MODE);
		rightDriveSlave1.set(rightDriveMaster.getDeviceID());
		rightDriveSlave2.changeControlMode(SLAVE_MODE);
		rightDriveSlave2.set(rightDriveMaster.getDeviceID());
	}

	/**
	 * Sets the Talon SRX's defaults (reversing, brake and control modes)
	 */
	private void setTalonDefaults() {
		reverseTalons(false);//Changing this didn't do anything, mathematically negated in drive command
		setBrakeMode(false);
		setCtrlMode(DEFAULT_CTRL_MODE);
	}
	
	private void resetGyro() {
		NavX.reset();
	}
	

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new Drive());
		
	}

}
