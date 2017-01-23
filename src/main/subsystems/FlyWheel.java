package main.subsystems;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
public class FlyWheel extends PIDSubsystem{

	public FlyWheel(String name, double p, double i, double d, double f, double period) {
		super(name, p, i, d, f, period);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

}
