package main.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import main.Constants;
import main.Robot;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TimedDrive;
import main.commands.drivetrain.TurnToAngle;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;

public class rightGearAuto extends CommandGroup implements Constants{
	public rightGearAuto() {
		addSequential(new TimedDrive(-0.5, 4.85));
		//***addSequential(new DriveDistance(3.85, kToleranceDisplacementDefault));//3.75
		//addSequential(new TimedDrive(0,0.1));
		//addSequential(new WaitCommand(1.5));
		//addSequential(new TurnToHeading(Robot.dt.getGyro().getYaw(), kToleranceDegreesDefault));
		//addSequential(new TimedDrive(0.5, 1.5));
		//addParallel(new WaitCommand(10));
		//***addSequential(new TurnToHeading(-45, kToleranceDegreesDefault));//-58.6
		//addSequential(new TimedDrive(0.5, 4));
		//***addSequential(new DriveDistance(4.5, kToleranceDisplacementDefault));//8//7.82
		//addSequential(new TurnToHeading(Robot.dt.getGyro().getYaw(), kToleranceDegreesDefault));
		//****addSequential(new TimedDrive(-0.4, 2.5));
		//addSequential(new DriveDistance(2, kToleranceDisplacementDefault));//8//7.82
		//addSequential(new DriveDistance(4, kToleranceDisplacementDefault));
		//addSequential(new TimedDrive)
		//addParallel(new WaitCommand(15));
		//****addSequential(new GearDown());
		//addSequential(new DriveDistance(-2, kToleranceDisplacementDefault));
		//*****addSequential(new TimedDrive(0.4, 2));
		//addParallel(new WaitCommand(5));
		//*****addSequential(new GearUp());
		
		//Drive distance doesn't like -distances, distances above 5, starting after being driven in teleop (encoder zeroing issue)
		

	}
}
