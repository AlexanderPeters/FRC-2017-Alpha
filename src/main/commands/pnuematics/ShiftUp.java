package main.commands.pnuematics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class ShiftUp extends CommandGroup {
    
    public  ShiftUp() {
    	addSequential(new Shift(DoubleSolenoid.Value.kReverse));
    	addSequential(new WaitCommand(0.5));
    	addSequential(new Shift(DoubleSolenoid.Value.kOff));
    }
}
