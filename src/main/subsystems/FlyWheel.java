package main.subsystems;

import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.vision.USBCamera;
import main.Constants;
import main.HardwareAdapter;

public class FlyWheel extends Subsystem implements Constants, HardwareAdapter{
	CameraServer camServer;
	UsbCamera cam;
	public FlyWheel() {
		camServer = CameraServer.getInstance();
		cam = camServer.startAutomaticCapture();
		cam.setFPS(30);
		cam.setResolution(640, 480);
		
		//CameraServer.getInstance().startAutomaticCapture();
		
	}
	public void speed() {
		
		shooter.changeControlMode(VELOCITY);
		shooter.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooter.setAllowableClosedLoopErr(flyWheelAllowableError);
		shooter.configEncoderCodesPerRev(codesPerRev);//Subject to change
		shooter.reverseSensor(false);
		shooter.configNominalOutputVoltage(+0.0f, -0.0f);
		shooter.configPeakOutputVoltage(+12.0f, -12.0f);
		shooter.setPID(flyWheelKP, flyWheelKI, flyWheelKD);
		shooter.set(flyWheelTargetVel);
		System.out.println("throttle " +  shooter.getOutputVoltage() + " encDirection" + shooter.getSpeed());
	}
	
	public double getSpeed() {
		return shooter.getSpeed();
	}
	public double getEncSpeed() {
		return shooter.getEncVelocity();
	}
	public double getError() {
		return shooter.getError();
	}

	public void speed(Double speed) {
		shooter.changeControlMode(PERCENT_VBUS_MODE);
		shooter.set(speed);
	}

	@Override
	protected void initDefaultCommand() {
		//ToDo
		
	}

}
