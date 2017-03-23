package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;

public class Target extends CommandGroup implements Constants{
	
	public Target() {
		addSequential(new TurnToGoal());
		addSequential(new DriveToGoal());
		addSequential(new TurnToGoal());
		addSequential(new DriveToGoal());
	}
}