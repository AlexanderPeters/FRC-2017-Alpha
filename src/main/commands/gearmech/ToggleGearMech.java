package main.commands.gearmech;

import edu.wpi.first.wpilibj.command.Command;
import main.Robot;

public class ToggleGearMech extends Command {
	public ToggleGearMech() {
		requires(Robot.pn);
	}
	protected void execute() {
    	Robot.pn.shiftGearMech();
    	
    }

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
