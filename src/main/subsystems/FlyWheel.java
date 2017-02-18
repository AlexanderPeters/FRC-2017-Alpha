package main.subsystems;

import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;

public class FlyWheel extends Subsystem implements Constants, HardwareAdapter{
	public void speed() {
		shooter.changeControlMode(VELOCITY);
		shooter.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooter.configNominalOutputVoltage(+0.0f, -0.0f);
		shooter.configPeakOutputVoltage(+12.0f, -12.0f);
		shooter.setPID(flyWheelKP, flyWheelKI, flyWheelKD);
		shooter.set(flyWheelTargetVel);
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
