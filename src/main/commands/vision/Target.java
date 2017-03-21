package main.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;
import main.Robot;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TurnToAngle;

public class Target extends CommandGroup implements Constants{
	public Target() {
		addSequential(new DriveDistance(4));
		addSequential(new TurnToAngle(30));
	}
	
}