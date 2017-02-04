package main.commands.gearmech;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import main.Constants;

/**
 *
 */
public class GearUp extends CommandGroup implements Constants{
    
    public  GearUp() {
    	addSequential(new ShiftGearMech(RET));
    	addSequential(new WaitCommand(0.5));
    	addSequential(new ShiftGearMech(OFF));
    }
}
