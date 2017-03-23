package main.subsystems;

import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;//NavX import
import Util.DriveHelper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import main.Constants;
import main.HardwareAdapter;
import main.Robot;
import main.commands.drivetrain.Drive;
import main.commands.pnuematics.ShiftDown;

public class DriveTrain extends Subsystem implements Constants, HardwareAdapter {
	private static boolean highGearState = false;
	private static AHRS NavX;
	private DriveHelper helper = new DriveHelper(7.5);
	private static RobotDrive driveTrain = new RobotDrive(leftDriveMaster, rightDriveMaster);
	private double smallTurnControllerRate, bigTurnControllerRate, distanceControllerRate;
	private PIDController smallTurnController;
	private PIDController bigTurnController;
	private PIDController distanceController;
	
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
		
		smallTurnController = new PIDController(turnInPlaceKPSmallAngle, turnInPlaceKISmallAngle, turnInPlaceKDSmallAngle, NavX, new PIDOutput() {
			public void pidWrite(double d) {
				smallTurnControllerRate = d + (kMinVoltageTurnSmallAngle*Math.signum(d))/10;
			}
		});
		
		bigTurnController = new PIDController(turnInPlaceKPBigAngle, turnInPlaceKIBigAngle, turnInPlaceKDBigAngle, NavX, new PIDOutput() {
			public void pidWrite(double d) {
				bigTurnControllerRate = d + (kMinVoltageTurnBigAngle*Math.signum(d))/10;
			}
		});
		
		distanceController = new PIDController(displacementKP, displacementKI, displacementKD, new PIDSource() {
			PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

			public double pidGet() {
				return (Robot.dt.getDistanceTraveledRight());
			}

			public void setPIDSourceType(PIDSourceType pidSource) {
				m_sourceType = pidSource;
			}

			public PIDSourceType getPIDSourceType() {
				return m_sourceType;
			}
		}, new PIDOutput() {public void pidWrite(double d) {
			distanceControllerRate = d;
		}
	});
		
		
	}
	public void driveVelocity(double throttle, double heading) {
		if(Robot.gameState == Robot.GameState.Teleop) 
			driveTrain.arcadeDrive(throttle, heading); 
	}

	public void driveStraight(double throttle) {
		double theta = NavX.getYaw();
		if(Math.signum(throttle) > 0) {
			//Make this PID Controlled
			driveTrain.arcadeDrive(helper.handleOverPower(throttle), helper.handleOverPower(theta * straightLineKP)); 
		}
		else {
			//Might be unnecessary but I think the gyro bearing changes if you drive backwards
			driveTrain.arcadeDrive(helper.handleOverPower(throttle), helper.handleOverPower(theta * straightLineKPReverse)); 
		}
				
	}
	public void driveDistanceSetPID(double p, double i, double d, double maxV) {
		distanceController.setPID(p, i, d);
		distanceController.setOutputRange(-maxV/10, maxV/10);
	}
	public void driveDistance(double distance, double tolerance) {
		if(highGearState)
			new ShiftDown();
		setBrakeMode(true);
		setCtrlMode(PERCENT_VBUS_MODE);
		//setVoltageDefaultsPID();
		
		distanceController.setInputRange(-20.0, +20.0);
		distanceController.setAbsoluteTolerance(tolerance);
		distanceController.setContinuous(true);
		distanceController.enable();
		distanceController.setSetpoint(distance);
		//System.out.println("r" + distanceControllerRate);
		this.driveVelocity(distanceControllerRate, 0.0);//Gyro code in drive straight I think is messed up
			
	}
	public void turnToBigAngleSetPID(double p, double i, double d, double maxV) {
		@SuppressWarnings("deprecation")
		double minVoltage = SmartDashboard.getDouble("Turning MinVoltage Big Angle", 0.0);
		bigTurnController.setPID(p, i, d);
		bigTurnController.setOutputRange(-(maxV-minVoltage)/10, (maxV-minVoltage)/10);
	}
			
	public void turnToBigAngle(double heading, double tolerance) {
		if(highGearState)
			new ShiftDown();
		setBrakeMode(true);
		setCtrlMode(PERCENT_VBUS_MODE);
				
		bigTurnController.setInputRange(-180.0f,  180.0f);
		bigTurnController.setAbsoluteTolerance(tolerance);
		bigTurnController.setContinuous(true);
		bigTurnController.enable();
		bigTurnController.setSetpoint(heading);
		this.driveVelocity(0.0, bigTurnControllerRate);
	}
	
	public void turnToSmallAngleSetPID(double p, double i, double d, double maxV) {
		@SuppressWarnings("deprecation")
		double minVoltage = SmartDashboard.getDouble("Turning MinVoltage Small Angle", 0.0);
		smallTurnController.setPID(p, i, d);
		smallTurnController.setOutputRange(-(maxV-minVoltage)/10, (maxV-minVoltage)/10);
	}
			
	public void turnToSmallAngle(double heading, double tolerance) {
		if(highGearState)
			new ShiftDown();
		setBrakeMode(true);
		setCtrlMode(PERCENT_VBUS_MODE);
				
		smallTurnController.setInputRange(-180.0f,  180.0f);
		smallTurnController.setAbsoluteTolerance(tolerance);
		smallTurnController.setContinuous(true);
		smallTurnController.enable();
		smallTurnController.setSetpoint(heading);
		this.driveVelocity(0.0, smallTurnControllerRate);
	}
	
	public double getDistanceAvg() {
		return (getDistanceTraveledLeft() + getDistanceTraveledRight())/2;
	}
	
	public void changeGearing(){
		highGearState = !highGearState;
	}
	
	public AHRS getGyro(){
		return NavX;
	}
	
	public int convertToEncoderTicks(double displacement) {//ft
		return (int) (((displacement / (wheelSize*Math.PI)) * conversionFactor));
	}
	public double getDistanceTraveledLeft() {//Feet
		return wheelSize*Math.PI*(getLeftEncoderPosition()/conversionFactor);
	}
	
	public double getDistanceTraveledRight() {//Feet
		//Removed - value and changed with reverseSensor() so that pid has correct feedback
		//System.out.println("r" +wheelSize*Math.PI*(getRightEncoderPosition()/conversionFactor));
		return wheelSize*Math.PI*(getRightEncoderPosition()/conversionFactor);
	}
	
	public double getLeftVelocity() {
		return leftDriveMaster.getEncVelocity() / wheelEncoderMult;
	}
	
	public double getRightVelocity() {
		return rightDriveMaster.getEncVelocity() / wheelEncoderMult;
	}
	
	public void resetGyro() {
		NavX.reset();
		NavX.zeroYaw();
	}
	public void resetEncoders() {
		leftDriveMaster.setEncPosition(0);//I'm gay
		rightDriveMaster.setEncPosition(0);//I'm gay
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
		return rightDriveMaster.getEncPosition();
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
		//leftDriveSlave2.enableBrakeMode(brake);
		rightDriveMaster.enableBrakeMode(brake);
		rightDriveSlave1.enableBrakeMode(brake);
		//rightDriveSlave2.enableBrakeMode(brake);
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
		//leftDriveSlave2.changeControlMode(SLAVE_MODE);
		//leftDriveSlave2.set(leftDriveMaster.getDeviceID());
		
		rightDriveMaster.changeControlMode(mode);
		rightDriveSlave1.changeControlMode(SLAVE_MODE);
		rightDriveSlave1.set(rightDriveMaster.getDeviceID());
		//rightDriveSlave2.changeControlMode(SLAVE_MODE);
		//rightDriveSlave2.set(rightDriveMaster.getDeviceID());
		
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
		leftDriveMaster.reverseSensor(true);//Check this later//was true
		rightDriveMaster.reverseSensor(true);//Check this later//was true
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
	
	/*private void setVoltageDefaultsPID() {
		leftDriveMaster.configNominalOutputVoltage(+0f, -0f);
		rightDriveMaster.configNominalOutputVoltage(+0f, -0f);
		leftDriveMaster.configPeakOutputVoltage(+6f, -6f);
		rightDriveMaster.configPeakOutputVoltage(+6f, -6f);
	}*/
	
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
		//setRampRate(12);//0-12v in 1 of a second //COMMENTED TO SEE IF THIS PREVENTS PID FROM FUNCTIONING
		reverseTalons(true);//Changing this didn't do anything, mathematically negated in drive command
		setBrakeMode(true);
		setCtrlMode(DEFAULT_CTRL_MODE);
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new Drive());
		
	}
		
}
