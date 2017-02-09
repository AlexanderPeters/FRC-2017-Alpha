package main.subsystems;

import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import Util.DriveHelper;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import main.Constants;
import main.HardwareAdapter;
import main.Robot;
//import main.Robot;
import main.commands.drivetrain.Drive;
import main.commands.pnuematics.ShiftDown;

//import Util.MathHelper;
//NavX import
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;



public class DriveTrain extends Subsystem implements Constants, HardwareAdapter, PIDOutput{
	private static boolean highGearState = false;
	private static AHRS NavX;
	private DriveHelper helper = new DriveHelper(7.5);
	private boolean hasBeenDrivingStriaghtWithThrottle;
	private static RobotDrive driveTrain = new RobotDrive(leftDriveMaster, rightDriveMaster);
	PIDController turnController;
	private static double rotateToAngleRate;
	
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
	public void driveTeleop(double throttle, double heading) {
		//if(helper.handleDeadband(heading, headingDeadband) != 0.0)
			driveWithHeading(throttle, heading);
		//else 
			//driveStraight(throttle);
	}
	
	private void driveWithHeading(double throttle, double heading) {
		if(Robot.gameState == Robot.GameState.Teleop) {//Friendly game state check
			
			Robot.robotState = Robot.RobotState.Driving;
			Robot.dt.setBrakeMode(false);
			
			hasBeenDrivingStriaghtWithThrottle = false;
			System.out.println(throttle);
			//helper.calculateThrottle(throttle), helper.calculateTurn(heading, highGearState)
			driveTrain.arcadeDrive(helper.handleOverPower(helper.handleJoystickHatingMe(helper.handleDeadband(throttle * driveThrottle, throttleDeadband))),
					helper.handleOverPower(helper.handleJoystickHatingMe(helper.handleDeadband(heading * turnThrottle, headingDeadband))));//helper.calculateThrottle(throttle)
		}
		
	}
	private void driveStraight(double throttle){
		if(Robot.gameState == Robot.GameState.Teleop) {//Friendly game state check
			
			Robot.robotState = Robot.RobotState.Driving;
			Robot.dt.setBrakeMode(false);
			
			if(!hasBeenDrivingStriaghtWithThrottle){
				resetGyro();
				System.out.println("ZEROING");
			}
			
			double theta = NavX.getYaw();
			System.out.println(theta);
			System.out.println("Here");
			if(Math.abs(helper.handleDeadband(throttle * driveThrottle, throttleDeadband)) > 0.0){//ABS fixed not driving backwards issue
				if(Math.signum(throttle) > 0) {
					//Make this PID Controlled
					driveTrain.arcadeDrive(helper.calculateThrottle(throttle), helper.handleOverPower(theta * -0.05)); 
				}
				else {
					//Might be unnecessary but I think the gyro bearing changes if you drive backwards
					driveTrain.arcadeDrive(helper.calculateThrottle(throttle), helper.handleOverPower(theta * -0.05)); 
				}
				
				hasBeenDrivingStriaghtWithThrottle = true;
			}
			else {
				hasBeenDrivingStriaghtWithThrottle = false;
			}
			
			if(theta <= 0.05)
				resetGyro();//Prevents accumulation of gyro drift (resets gyro noise if robot is on course)
		}
	}
	
	public void driveLooperControl(double leftThrottle, double rightThrottle) {
		if(Robot.gameState == Robot.GameState.Autonomous) {//Friendly game state check
			
			Robot.robotState = Robot.RobotState.Driving;
			Robot.dt.setBrakeMode(true);
			new ShiftDown();
			
			driveTrain.tankDrive(helper.handleOverPower(leftThrottle), helper.handleOverPower(rightThrottle));
		}
		
	}
	
	public void turnToHeading(double heading) {
		new ShiftDown();
		setBrakeMode(true);
		resetGyro();
		turnController = new PIDController(turnInPlaceKP, turnInPlaceKI, turnInPlaceKD, turnInPlaceKF, NavX, this);
		turnController.setInputRange(-180.0f,  180.0f);
	    turnController.setOutputRange(-1.0, 1.0);
	    turnController.setAbsoluteTolerance(kToleranceDegrees);
	    turnController.setContinuous(true);
		turnController.enable();
		driveTrain.arcadeDrive(0.0, rotateToAngleRate);
		
		
	}
	
	public void driveDisplacement(double displacement) {
		new ShiftDown();
		setBrakeMode(true);
		setCtrlMode(POSITION); //Change control mode of talon, default is PercentVbus (-1.0 to 1.0)
		
		leftDriveMaster.setPID(leftDisplacementKP, leftDisplacementKI, leftDisplacementKD); 
		leftDriveMaster.setAllowableClosedLoopErr(leftDisplacementTolerance);
		
		rightDriveMaster.setPID(rightDisplacementKP, rightDisplacementKI, rightDisplacementKD); 
		rightDriveMaster.setAllowableClosedLoopErr(rightDisplacementTolerance);
		
		leftDriveMaster.enableControl(); //Enable PID control on the talon
		rightDriveMaster.enableControl(); //Enable PID control on the talon
		
		leftDriveMaster.set(convertToEncoderTicks(displacement));
		rightDriveMaster.set(convertToEncoderTicks(displacement));
	}
	
	public void changeGearing(){
		highGearState = !highGearState;
	}
	
	public AHRS getGyro(){
		return NavX;
	}
	
	public double getLeftEncoderPosition() {
		return leftDriveMaster.getEncPosition();
	}
	
	public double getRightEncoderPosition() {
		return rightDriveMaster.getEncPosition();
	}
	
	public double convertToEncoderTicks(double dispacement) {//ft
		return 0;
	}
	
	public double getDistanceTraveledRight() {
		return 0;
	}
	
	public double getLeftEncoderVelocity() {
		return leftDriveMaster.getEncVelocity();
	}
	
	public double getRightEncoderVelocity() {
		return rightDriveMaster.getEncVelocity();
	}
	
	public void resetGyro() {
		NavX.reset();
		NavX.zeroYaw();
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
	 * Set's the Talon SRX's feedback device
	 * 
	 */
	private void setFeedBackDefaults() {
		leftDriveMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightDriveMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		leftDriveMaster.configEncoderCodesPerRev(codesPerRev);
		rightDriveMaster.configEncoderCodesPerRev(codesPerRev);
		leftDriveMaster.reverseSensor(false);
		rightDriveMaster.reverseSensor(false);
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
		setRampRate(18);//0-12v in 3/4 of a second
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
