package main.commands.drivetrain;

import Util.PlanPath;
import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;

public class DriveTo extends CommandGroup implements Constants {
	private PlanPath planner;

	public DriveTo(double dispX, double dispY) {
		planner = new PlanPath(dispX, dispY);
		addSequential(new TurnToHeading(planner.getAngle(), kToleranceDegreesDefault));
		addSequential(new DriveDistance(planner.getHypotenuse(), kToleranceDisplacementDefault));
		addSequential(new TurnToHeading(-planner.getAngle(), kToleranceDegreesDefault));
		addSequential(new DriveDistance(planner.getDisplacementToFinalDest(distanceBetweenRobotAndGearPeg),
				kToleranceDisplacementDefault));

	}

}
