package main.commands.pnuematics;

import edu.wpi.first.wpilibj.command.Command;
import main.Robot;

public class ToggleCompressor extends Command{
	
	public ToggleCompressor(){
		Robot.pn.toggleComp();
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
