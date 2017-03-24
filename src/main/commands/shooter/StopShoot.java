package main.commands.shooter;

import edu.wpi.first.wpilibj.command.CommandGroup;
import main.commands.stirrer.StirOff;

public class StopShoot extends CommandGroup {
	public StopShoot() {
		addParallel(new FlyWheelOff());
		addParallel(new StirOff());
	}

}
