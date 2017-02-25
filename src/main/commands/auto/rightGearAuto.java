package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TurnToHeading;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;

public class rightGearAuto extends CommandGroup implements Constants{
	public rightGearAuto() {
		addSequential(new DriveDistance(3.75, kToleranceDisplacementDefault));
		addSequential(new TurnToHeading(-58.6, kToleranceDegreesDefault));
		addSequential(new DriveDistance(7.32, kToleranceDisplacementDefault));
		addSequential(new GearDown());
		addSequential(new DriveDistance(-2, kToleranceDisplacementDefault));
		addSequential(new GearUp());

	}
}
