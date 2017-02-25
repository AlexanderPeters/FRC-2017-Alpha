package main.commands.drivetrain;

import Util.PlanPathToGear;
import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;

public class DriveAndPlaceGear extends CommandGroup implements Constants {
	private PlanPathToGear planner;
		
	public DriveAndPlaceGear(double dispX, double dispY) {
		planner = new PlanPathToGear(dispX, dispY);
		addSequential(new TurnToHeading(planner.getAngle(), kToleranceDegreesDefault));
		addSequential(new DriveDistance(planner.getHypotenuse(), kToleranceDisplacementDefault));
		addSequential(new TurnToHeading(-planner.getAngle(), kToleranceDegreesDefault));
		addSequential(new DriveDistance(planner.getDisplacementToGear(distanceBetweenRobotAndGearPeg), kToleranceDisplacementDefault));
		addSequential(new GearDown());
		addSequential(new DriveDistance(-1, kToleranceDisplacementDefault));
		addSequential(new GearUp());
	}
	
	

}
