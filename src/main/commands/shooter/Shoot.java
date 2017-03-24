package main.commands.shooter;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import main.commands.stirrer.Stir;

public class Shoot extends CommandGroup {
	public Shoot(double shooterThrottle, double stirThrottle) {
		addParallel(new FlyWheelForward(shooterThrottle));
		addSequential(new WaitCommand(1));
		addSequential(new Stir(stirThrottle));
	}
}