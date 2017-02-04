package main.commands.gearmech;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import main.Constants;

/**
 *
 */
public class GearDown extends CommandGroup implements Constants{
    
    public  GearDown() {
    	addSequential(new ShiftGearMech(EXT));
    	addSequential(new WaitCommand(0.5));
    	addSequential(new ShiftGearMech(OFF));
    }
}
