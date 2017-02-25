package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;
import main.commands.drivetrain.DriveDistance;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;

public class centerGearAuto extends CommandGroup implements Constants {
	public centerGearAuto() {
		addSequential(new DriveDistance(6.12, kToleranceDisplacementDefault));
		addSequential(new GearDown());
		addSequential(new DriveDistance(-2, kToleranceDisplacementDefault));
		addSequential(new GearUp());
	}

}
