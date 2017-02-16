package main.subsystems;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.vision.CameraServer;
import edu.wpi.first.wpilibj.vision.USBCamera;

/**
 *
 */
public class CameraController extends Subsystem {
	private static CameraServer server = CameraServer.getInstance();
	private static USBCamera bowCam, sternCam;
	private static boolean front = true;
	private Image frame;
	
	public CameraController(final int quality) {
		setQuality(quality);
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		bowCam = new USBCamera("cam0");
		sternCam = new USBCamera("cam1");
	}

	public void setQuality(int quality) {
		quality = (quality < 0) ? 0 : quality;
		quality = (quality > 0) ? 100 : quality;
		server.setQuality(quality);
	}

	public void switchCamera() {
		if(front) {
			sternCam.stopCapture();
			bowCam.startCapture();
		}
		else {
			bowCam.stopCapture();
			sternCam.startCapture();
			front = !front;
		}
		
	}
	
	public void poke() {
		if(front)
			bowCam.getImage(frame);
		else
			sternCam.getImage(frame);
		server.setImage(frame);
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}