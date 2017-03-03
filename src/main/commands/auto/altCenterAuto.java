package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TurnToHeading;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;

public class altCenterAuto extends CommandGroup implements Constants{
	public altCenterAuto() {
		addSequential(new DriveDistance(4.791, kToleranceDisplacementDefault));
		addSequential(new GearDown());
		addSequential(new TurnToHeading(0, kToleranceDegreesDefault));
		addSequential(new DriveDistance(-2, kToleranceDisplacementDefault));
		addSequential(new GearUp());
		
	}

}
