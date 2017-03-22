package main.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import main.Constants;
import main.Robot;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TurnToAngle;

public class Target extends CommandGroup implements Constants{
	private double distanceToGoal;
	public Target() {
		//Everything in ft
		//if(Robot.comms.getTargetFound()) {
			distanceToGoal = Robot.comms.getRange()*Math.cos(cameraAngle * Math.PI/180);
			//addSequential(new DriveDistance(desiredDistanceToGoal-distanceToGoal));
			//addSequential(new DriveDistance(desiredDistanceToGoal-distanceToGoal));
			System.out.println("Angle to turn to " + Robot.comms.getBearing());
			addSequential(new TurnToAngle(Robot.comms.getBearing()));
			addSequential(new TurnToAngle(Robot.comms.getBearing()));
		//}
	}
}