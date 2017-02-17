package main.subsystems;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.vision.CameraServer;
import edu.wpi.first.wpilibj.vision.USBCamera;

/**
 *
 */
public class DriverCamera extends Subsystem {
	private static CameraServer server = CameraServer.getInstance();
	private static USBCamera gearCam, ropeCam;
	private static boolean camBool = true;
	private Image frame;
	
	public DriverCamera(final int quality) {
		setQuality(quality);
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		gearCam = new USBCamera("cam0");
		ropeCam = new USBCamera("cam1");
	}

	public void setQuality(int quality) {
		quality = (quality < 0) ? 0 : quality;
		quality = (quality > 0) ? 100 : quality;
		server.setQuality(quality);
	}

	public void switchCamera() {
		if(camBool) {
			gearCam.startCapture();
			ropeCam.stopCapture();
		}
		else {
			gearCam.stopCapture();
			ropeCam.startCapture();
			camBool = !camBool;
		}
		
	}
	
	public void poke() {
		if(camBool)
			gearCam.getImage(frame);
		else
			ropeCam.getImage(frame);
		server.setImage(frame);
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}