package lib;

import edu.wpi.first.wpilibj.command.Command;

public abstract class TimedCommand extends Command{
    
    private Mechanism mechanism;
    
    public TimedCommand(Mechanism mechanism, double time){
        super(time);
        this.mechanism = mechanism;
        requires(mechanism);
    }
    
    @Override
    protected void execute() {
        //do nothing
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }


    @Override
    protected void interrupted() {
        throw new UnsupportedOperationException("Not supported yet.");
    }
    
    @Override
    protected void end() {
        mechanism.stop();
    }
    
}