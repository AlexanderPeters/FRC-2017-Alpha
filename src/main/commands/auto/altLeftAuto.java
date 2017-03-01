package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TimedDrive;
import main.commands.drivetrain.TurnToHeading;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;

public class altLeftAuto extends CommandGroup implements Constants{
	public altLeftAuto() {
		addSequential(new DriveDistance(3.75, kToleranceDisplacementDefault));
		addSequential(new TurnToHeading(45, 0.5));//Better turning tolerance
		addSequential(new DriveDistance(4.82, kToleranceDisplacementDefault));
		//addSequential(new TurnToHeading(0, kToleranceDegreesDefault)); Use if you need to reset and then run DriveDisplacement again
		addSequential(new GearDown());
		addSequential(new TimedDrive(0.4, 2));
		addSequential(new GearUp());
	}

}
