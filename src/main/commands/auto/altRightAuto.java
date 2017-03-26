package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TimedDrive;
import main.commands.drivetrain.TurnToAngle;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;

public class altRightAuto extends CommandGroup implements Constants {
	public altRightAuto() {
		addSequential(new DriveDistance(1.51, kToleranceDisplacementDefault));
		//addSequential(new DriveDistance(1, kToleranceDisplacementDefault));
		addSequential(new TurnToAngle(-45, 0.5));//Better turning tolerance
		addSequential(new DriveDistance(8.68, kToleranceDisplacementDefault));
		//addSequential(new DriveDistance(1.188, kToleranceDisplacementDefault));
		//addSequential(new TurnToHeading(0, kToleranceDegreesDefault)); Use if you need to reset and then run DriveDisplacement again
		addSequential(new GearDown());
		addSequential(new TimedDrive(0.55, 2));
		addSequential(new GearUp());
	}

}
