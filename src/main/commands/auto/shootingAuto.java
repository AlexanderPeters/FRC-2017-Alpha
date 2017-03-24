package main.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import main.Constants;
import main.commands.drivetrain.TimedDrive;
import main.commands.drivetrain.TurnToAngle;
import main.commands.shooter.FlyWheelForTime;
import main.commands.stirrer.StirForTime;

public class shootingAuto extends CommandGroup implements Constants {
	public shootingAuto() {
		addParallel(new FlyWheelForTime(shooterForward, 6));
		addSequential(new WaitCommand(1));
		addSequential(new StirForTime(stirrerMotorForward, 5));
		
		if(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
			addSequential(new TimedDrive(-0.5, 0.3, 2));
			addSequential(new TurnToAngle(70));//Assuming a real drift of 20 deg calculated was 30.9
		}
		else {
			addSequential(new TimedDrive(-0.5, -0.3, 2));
			addSequential(new TurnToAngle(-70));//Assuming a real drift of 20 deg calculated was 30.9
		}
		addSequential(new TimedDrive(-0.5, 5));
	}

}
