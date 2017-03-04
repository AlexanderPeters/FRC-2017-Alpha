package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TimedDrive;
import main.commands.drivetrain.TurnToHeading;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;

public class leftGearAuto extends CommandGroup implements Constants {
	public leftGearAuto() {
		addSequential(new TimedDrive(-0.5, 4.85));
		//addSequential(new DriveDistance(3.85, kToleranceDisplacementDefault));//3.75
		//addSequential(new TurnToHeading(45, kToleranceDegreesDefault));//58.6
		//addSequential(new DriveDistance(4.5, kToleranceDisplacementDefault));//8//7.82
		//addSequential(new TimedDrive(-0.4, 2.5));
		//addSequential(new GearDown());
		//addSequential(new TimedDrive(0.4, 2));
		//addSequential(new GearUp());
		
		//Drive distance doesn't like -distances, distances above 5, starting after being driven in teleop (encoder zeroing issue)

	}
}
