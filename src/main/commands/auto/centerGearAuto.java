package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import main.Constants;
import main.commands.drivetrain.TimedDrive;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;

public class centerGearAuto extends CommandGroup implements Constants {
	public centerGearAuto() {
		addSequential(new TimedDrive(-0.5, 4.85));
		addSequential(new GearDown());
		addSequential(new WaitCommand(1));
		addSequential(new TimedDrive(0.55, 2));
		addSequential(new GearUp());
	}

}
