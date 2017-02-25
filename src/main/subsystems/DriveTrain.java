package main.subsystems;

import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;//NavX import
import Util.DriveHelper;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import main.Constants;
import main.HardwareAdapter;
import main.Robot;
import main.commands.drivetrain.Drive;
import main.commands.pnuematics.ShiftDown;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends Subsystem implements Constants, HardwareAdapter, PIDOutput{
	private static boolean highGearState = false;
	private static AHRS NavX;
	private DriveHelper helper = new DriveHelper(7.5);
	private boolean hasBeenDrivingStriaghtWithThrottle;
	private static RobotDrive driveTrain = new RobotDrive(leftDriveMaster, rightDriveMaster);
	private static double rotateToAngleRate;
	private PIDController turnController;
		
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
		resetSensors();//Must happen after NavX is instantiated!
		turnController = new PIDController(turnInPlaceKP, turnInPlaceKI, turnInPlaceKD, turnInPlaceKF, NavX, this);
		
		
	}
	public void driveTeleop(double throttle, double heading) {
		if(helper.handleDeadband(heading, headingDeadband) != 0.0) {
			driveWithHeading(throttle, heading);
			Robot.robotState = Robot.RobotState.Driving;
		}
		//double mini = (pdp.getCurrent(12) + pdp.getCurrent(13)) / 2;
		//double cim = (pdp.getCurrent(0) + pdp.getCurrent(1) + pdp.getCurrent(2) + pdp.getCurrent(3)) / 4;
		//System.out.println("Cim: " + cim + ", Mini: " + mini + ", Total: " + pdp.getTotalCurrent());
		//System.out.printboolean(helper.handleDeadband(throttle, throttleDeadband)) > 0.0);
		else if(Math.abs(helper.handleDeadband(throttle, 0.25)) > 0.0) {
			driveStraight(throttle);
			Robot.robotState = Robot.RobotState.Driving;
		}
		else if(Robot.robotState != Robot.RobotState.Climbing)
			Robot.robotState = Robot.RobotState.Neither;
		//System.out.println("left " + getDistanceTraveledLeft() + " right " + getDistanceTraveledRight());
		//System.out.print(Math.abs(helper.handleDeadband(throttle, 0.2)) > 0.0);
		//System.out.println(Math.abs(helper.handleDeadband(throttle, 0.2)));
	}
	
	private void driveWithHeading(double throttle, double heading) {
		if(Robot.gameState == Robot.GameState.Teleop) {//Friendly game state check
			
			Robot.dt.setBrakeMode(false);
			setCtrlMode(PERCENT_VBUS_MODE);
			
			hasBeenDrivingStriaghtWithThrottle = false;
			//System.out.println(throttle);
			//helper.calculateThrottle(throttle), helper.calculateTurn(heading, highGearState)
			driveTrain.arcadeDrive(helper.handleOverPower(helper.handleJoystickHatingMe(helper.handleDeadband(throttle * driveThrottle, throttleDeadband))),
					helper.handleOverPower(helper.handleJoystickHatingMe(helper.handleDeadband(heading * turnThrottle, headingDeadband))));//helper.calculateThrottle(throttle)
			//System.out.println("Gyro" + NavX.getYaw());
		}
		
	}
	public void driveStraight(double throttle) {
		if(Robot.gameState == Robot.GameState.Teleop) {//Friendly game state check
			
			Robot.dt.setBrakeMode(false);
			setCtrlMode(PERCENT_VBUS_MODE);
			
			if(!hasBeenDrivingStriaghtWithThrottle){
				resetGyro();
				//System.out.println("ZEROING");
			}
			
			double theta = NavX.getYaw();
			System.out.println(theta);
			//System.out.println(theta);
			//System.out.println("Here");
			if(Math.abs(helper.handleDeadband(throttle * driveThrottle, throttleDeadband)) > 0.0){//ABS fixed not driving backwards issue
				if(Math.signum(throttle) > 0) {
					//Make this PID Controlled
					driveTrain.arcadeDrive(helper.calculateThrottle(throttle), helper.handleOverPower(theta * straightLineKP)); 
				}
				else {
					//Might be unnecessary but I think the gyro bearing changes if you drive backwards
					driveTrain.arcadeDrive(helper.calculateThrottle(throttle), helper.handleOverPower(theta * straightLineKPReverse)); 
				}
				
				hasBeenDrivingStriaghtWithThrottle = true;
			}
			else {
				hasBeenDrivingStriaghtWithThrottle = false;
			}
			//System.out.println("Straight Gyro" + NavX.getYaw());

			
			//if(theta <= 0.05)
				//resetGyro();//Prevents accumulation of gyro drift (resets gyro noise if robot is on course)
		}
	}
	
	public void driveLooperControl(double leftThrottle, double rightThrottle) {
		if(Robot.gameState == Robot.GameState.Autonomous) {//Friendly game state check
			
			new ShiftDown();
			Robot.dt.setBrakeMode(true);
			setCtrlMode(PERCENT_VBUS_MODE);

			
			driveTrain.tankDrive(-1*helper.handleOverPower(leftThrottle), -1*helper.handleOverPower(rightThrottle));
		}
		
	}
	
	public void turnToHeading(double heading, double tolerance) {
		if(highGearState)
			new ShiftDown();
		setBrakeMode(true);
		setCtrlMode(PERCENT_VBUS_MODE);
				
		turnController.setInputRange(-180.0f,  180.0f);
	    turnController.setOutputRange(-1.0, 1.0);
	    turnController.setAbsoluteTolerance(tolerance);
	    turnController.setContinuous(true);
		turnController.enable();
		turnController.setSetpoint(heading);
		driveTrain.arcadeDrive(0.0, rotateToAngleRate);
		
		
	}
	
	public void driveDisplacement(double displacement, double tolerance) {//feet, feet
		if(highGearState)
			new ShiftDown();
		setBrakeMode(true);
		setCtrlMode(POSITION); //Change control mode of talon, default is PercentVbus (-1.0 to 1.0)
		
		leftDriveMaster.setPID(displacementKP, displacementKI, displacementKD); 
		leftDriveMaster.setAllowableClosedLoopErr(convertToEncoderTicks(tolerance));
		
		rightDriveMaster.setPID(displacementKP, displacementKI, displacementKD); 
		rightDriveMaster.setAllowableClosedLoopErr(convertToEncoderTicks(tolerance));
		
		leftDriveMaster.enableControl(); //Enable PID control on the talon
		rightDriveMaster.enableControl(); //Enable PID control on the talon
		//rightDriveMaster.setFeedbackDevice(leftDriveMaster.getEncPosition());
		
		leftDriveMaster.setSetpoint(-convertToEncoderTicks(displacement));
		rightDriveMaster.setSetpoint(convertToEncoderTicks(displacement));
		System.out.println(getDistanceTraveledLeft()+ " " +getDistanceTraveledRight()+ " " + leftDriveMaster.getEncPosition()+ " " + rightDriveMaster.getEncPosition());

		
	}
	
	public void changeGearing(){
		highGearState = !highGearState;
	}
	
	public AHRS getGyro(){
		return NavX;
	}
	
	public int convertToEncoderTicks(double displacement) {//ft
		return (int) (((displacement / (wheelSize*Math.PI)) * codesPerRev));
	}
	public double getDistanceTraveledLeft() {//Feet
		return wheelSize*Math.PI*(getLeftEncoderPosition()/codesPerRev);
	}
	
	public double getDistanceTraveledRight() {//Feet
		return -wheelSize*Math.PI*(getRightEncoderPosition()/codesPerRev);
	}
	
	public double getLeftVelocity() {
		return leftDriveMaster.getEncVelocity() / wheelEncoderVelMult;
	}
	
	public double getRightVelocity() {
		return rightDriveMaster.getEncVelocity() / wheelEncoderVelMult;
	}
	
	public void resetGyro() {
		NavX.reset();
		NavX.zeroYaw();
	}
	public void resetEncoders() {
		leftDriveMaster.setPosition(0);
		rightDriveMaster.setPosition(0);
	}
	public void resetSensors() {
		resetGyro();
		resetEncoders();
	}
	
	
	
	
	/*******************
	 * SUPPORT METHODS *
	 *******************/
	private double getLeftEncoderPosition() {
		return leftDriveMaster.getEncPosition();
	}
	
	private double getRightEncoderPosition() {
		return rightDriveMaster.getEncPosition();//F*#k Me
	}
	
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
	 * Set's the Talon SRX's feedback device
	 * 
	 */
	private void setFeedBackDefaults() {
		leftDriveMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightDriveMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		leftDriveMaster.configEncoderCodesPerRev(codesPerRev);
		rightDriveMaster.configEncoderCodesPerRev(codesPerRev);
		//leftDriveMaster.reverseSensor(true);//Check this later
		//rightDriveMaster.reverseSensor(true);//Check this later
	}
	
	/**
	 * Sets the Talon SRX's voltage defaults (Serves to help keep the drivetrain consistent)
	 */
	private void setVoltageDefaults() {
		leftDriveMaster.configNominalOutputVoltage(+0f, -0f);
		rightDriveMaster.configNominalOutputVoltage(+0f, -0f);
		leftDriveMaster.configPeakOutputVoltage(+12f, -12f);
		rightDriveMaster.configPeakOutputVoltage(+12f, -12f);
	}
	
	/**
	 * Sets the Talon SRX's voltage ramp rate (Smooth's acceleration (units in volts/sec))
	 */
	private void setRampRate(double ramp) {
		leftDriveMaster.setVoltageCompensationRampRate(ramp);
		rightDriveMaster.setVoltageCompensationRampRate(ramp);
	}

	/**
	 * Sets the Talon SRX's defaults (reversing, brake and control modes)
	 */
	private void setTalonDefaults() {
		setFeedBackDefaults();
		setVoltageDefaults();
		setRampRate(12);//0-12v in 1 of a second
		reverseTalons(false);//Changing this didn't do anything, mathematically negated in drive command
		setBrakeMode(false);
		setCtrlMode(DEFAULT_CTRL_MODE);
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new Drive());
		
	}
	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;		
	}

}
