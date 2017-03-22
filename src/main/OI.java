package main;

import lib.joystick.XboxController;
import main.commands.climber.WinchForward;
import main.commands.climber.WinchOff;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TurnToAngle;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;
import main.commands.intake.IntakeForward;
import main.commands.intake.IntakeOff;
import main.commands.pnuematics.ShiftDown;
import main.commands.pnuematics.ShiftUp;
import main.commands.shooter.FlyWheelForward;
import main.commands.shooter.FlyWheelOff;
import main.commands.stirrer.Stir;
import main.commands.vision.Target;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements Constants, HardwareAdapter {
	
	public OI() {
		check();
	}
	
	public static XboxController getXbox (){
		return xbox;
	}

	public void check(){
		xbox.leftBumper.whenPressed(new ShiftUp());
		xbox.leftBumper.whenReleased(new ShiftDown());
		xbox.rightBumper.whenPressed(new GearDown());
		xbox.rightBumper.whenReleased(new GearUp());
		//xbox.start.whenPressed(new TurnToAngle(30));
		xbox.start.whenPressed(new TurnToAngle (55));
		//xbox.select.whenPressed(new TurnToAngle(-3));
		xbox.select.whenReleased(new TurnToAngle(20));
		xbox.leftTrigger.whenPressed(new TurnToAngle(Robot.comms.getBearing()));
		//xbox.leftTrigger.whenPressed(new TurnToTarget());
		
		xbox.a.whileHeld(new IntakeForward());
		xbox.a.whenReleased(new IntakeOff());
		xbox.x.whileHeld(new WinchForward(true));
		xbox.b.whileHeld(new WinchForward(false));
		xbox.b.whenReleased(new WinchOff());
		xbox.y.whileHeld(new FlyWheelForward(shooterForward));
		xbox.y.whileHeld(new Stir(stirrerMotorReverse));
		xbox.y.whenReleased(new FlyWheelOff());
		xbox.y.whenReleased(new Stir(0));
		xbox.rightTrigger.whileHeld(new FlyWheelForward(0.8));
		xbox.rightTrigger.whileHeld(new Stir(stirrerMotorReverse));
		xbox.rightTrigger.whenReleased(new FlyWheelOff());
		xbox.rightTrigger.whenReleased(new Stir(0));
	}
}

