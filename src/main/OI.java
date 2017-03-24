package main;

import lib.joystick.XboxController;
import main.commands.climber.WinchForward;
import main.commands.climber.WinchOff;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.Target;
import main.commands.drivetrain.TurnToAngle;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;
import main.commands.intake.IntakeForward;
import main.commands.intake.IntakeOff;
import main.commands.pnuematics.ShiftDown;
import main.commands.pnuematics.ShiftUp;
import main.commands.shooter.FlyWheelForward;
import main.commands.shooter.FlyWheelOff;
import main.commands.shooter.Shoot;
import main.commands.shooter.StopShoot;
import main.commands.stirrer.Stir;
import main.commands.stirrer.StirForTime;

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
		//xbox.start.whenPressed(new Target());
		//xbox.select.whenPressed(new TurnToAngle(-3));
		//xbox.select.whenReleased(new TurnToAngle(20));
		
		//xbox.leftTrigger.whenPressed(new TurnToAngle(mmm));
		//xbox.select.whenPressed(new StirForTime(1,3));
		//xbox.leftTrigger.whenPressed(new Target());
		
		xbox.a.whileHeld(new IntakeForward());
		xbox.a.whenReleased(new IntakeOff());
		xbox.x.whileHeld(new WinchForward(true));
		xbox.b.whileHeld(new WinchForward(false));
		xbox.b.whenReleased(new WinchOff());
		//xbox.y.whileHeld(new FlyWheelForward(shooterForward));
		xbox.y.whileHeld(new Stir(stirrerMotorReverse));
		//xbox.y.whenReleased(new FlyWheelOff());
		xbox.y.whenReleased(new Stir(stirrerMotorOff));
		xbox.rightTrigger.whileHeld(new Shoot(shooterForward, stirrerMotorForward));
		xbox.rightTrigger.whenReleased(new StopShoot());
		//xbox.rightTrigger.whileHeld(new FlyWheelForward(shooterForward));
		//xbox.rightTrigger.whileHeld(new Stir(stirrerMotorForward));
		//xbox.rightTrigger.whenReleased(new FlyWheelOff());
		//xbox.rightTrigger.whenReleased(new Stir(stirrerMotorOff));
	}
}

