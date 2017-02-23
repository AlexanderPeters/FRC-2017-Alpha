package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;

public class Indexer extends Subsystem implements Constants, HardwareAdapter{
	
	protected void initDefaultCommand() {
		
	}
	
	public void set(double angle) {
		shooterIndexer.setAngle(angle);
	}
	
	

}
