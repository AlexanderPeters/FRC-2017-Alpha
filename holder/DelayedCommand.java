package lib;

import java.lang.reflect.Constructor;

import edu.wpi.first.wpilibj.command.Command;

public class DelayedCommand{
	private boolean timedBoolean;
	private TimeDelayedBoolean myBool;
	
	public DelayedCommand(double time, Class object){
		myBool = new TimeDelayedBoolean(time);
		while(true){
			if(myBool.get()){
				Class<?> itsclass = object.getClass();
				new itsclass();
				break;
			}
				
		}
		
		
	}

}
