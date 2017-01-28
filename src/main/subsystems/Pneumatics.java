package main.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.Robot;

public class Pneumatics extends Subsystem implements Constants {

	/*****************
	 * INSTANCE DATA *
	 *****************/
	private DoubleSolenoid shifter;
	private DoubleSolenoid gearMech;
	private Compressor comp;
	private boolean gearMechState = true;

	/**
	 * Constructor
	 */
	public Pneumatics() {
		shifter = new DoubleSolenoid(1, SHIFTER_EXT, SHIFTER_RET);
		gearMech = new DoubleSolenoid(1, GEAR_EXT, GEAR_RET);
		comp = new Compressor(Constants.PCM_Port);
		comp.setClosedLoopControl(true);
		shifter.set(EXT);
		shifter.set(OFF);
		gearMech.set(EXT);
		gearMech.set(OFF);


	}

	
	
	/*******************
	 * COMMAND METHODS *
	 *******************/

	/**
	 * Shifts the gearbox from the different gears
	 * 
	 * @param v - Desired shifting value (Uses default shifting values)
	 */
	public void shift(DoubleSolenoid.Value v) {
		shifter.set(v);
		Robot.dt.changeGearing();
	}
	
	public void shiftGearMech(DoubleSolenoid.Value v) {
		gearMech.set(v);
	}
	public void toggleGearMech() {
		if(gearMechState)
			gearMech.set(EXT);
		else
			gearMech.set(RET);
		gearMechState = !gearMechState;
	}

	/**
	 * Toggles the compressor (ON/OFF)
	 */
	public void toggleComp() {
		if (comp.enabled())
			comp.stop();
		else
			comp.start();
	}

	/*******************
	 * DEFAULT METHODS *
	 *******************/
	public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}
