package controllers;


import java.io.IOException;
import lib.Loop;
import main.Constants;
import main.Robot;

public class UDPController implements Loop, Constants{
	
	

	@Override
	public void onStart() {
		//no-op
	}

	@Override
	public void onLoop() {
		System.out.println("here");

		try {
			Robot.comms.poke();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public void onStop() {
		//no-op
	}
	

}
