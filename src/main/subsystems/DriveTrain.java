package main.subsystems;

//Example import
import edu.wpi.first.wpilibj.CANTalon;

//Needed for sure
import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.OI;
import main.Robot;
import main.commands.drivetrain.Drive;

/**
 *
 */
public class DriveTrain extends Subsystem implements Constants {
	public static DriveTrain instance;

	//Instantiate devices here
	
	private DriveTrain() {
		//Initialize devices here
	}

	//Used to initialize the class upon startup
	public static DriveTrain getInstance() {
		if (instance == null) {
			instance = new DriveTrain();
		}
		return instance;
	}

	public void drive(){
		//Actual driving code goes here
	}
	
	//Use this command if no other command is currently controlling that subsystem
	//such as an autonomous command
	public void initDefaultCommand() {
		setDefaultCommand(new Drive());
	}
}