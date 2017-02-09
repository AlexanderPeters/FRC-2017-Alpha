package main.commands.driverCam;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import main.Constants;
import main.Robot;

/**
 *
 */
public class StreamCamera extends Command implements Constants{
	private Thread captureThread;

	public StreamCamera() {
		requires(Robot.cc);
	}

	@Override
	protected void initialize() {
		captureThread = new Thread(() -> {
			while (true) {
				Robot.cc.stream();
				System.out.println(1 / (double) fps);
				Timer.delay(1 / (double) fps);
			}
		});
		captureThread.setName("Camera Capture Thread");
		captureThread.start();
	}

	@Override
	protected void execute() {
		Robot.cc.stream();

	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@SuppressWarnings("deprecation")
	@Override
	protected void end() {
		captureThread.stop();
	}

	@Override
	protected void interrupted() {
		end();
	}

}