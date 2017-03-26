package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import main.Constants;
import main.commands.drivetrain.TimedDrive;
import main.commands.drivetrain.TurnToAngle;
import main.commands.shooter.FlyWheelForTime;
import main.commands.stirrer.StirForTime;

public class shootingAutoRedAlliance extends CommandGroup implements Constants {
	public shootingAutoRedAlliance() {
		addParallel(new FlyWheelForTime(shooterForward, 6));
		addSequential(new WaitCommand(1));
		addSequential(new StirForTime(stirrerMotorForward, 5));
		addSequential(new TimedDrive(-0.5, -kMinVoltageTurnBigAngle, 0.65));
		//addSequential(new TurnToAngle(-52));//Assuming a real drift of 20 deg calculated was 47.7
		addSequential(new TimedDrive(-0.5, 5)); 
	}

}
