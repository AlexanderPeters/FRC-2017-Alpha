package main.commands.pnuematics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import main.Constants;

/**
 *
 */
public class ShiftDown extends CommandGroup implements Constants{
    
    public  ShiftDown() {
    	addSequential(new Shift(EXT));
    	addSequential(new WaitCommand(0.5));
    	addSequential(new Shift(OFF));
    }
}
