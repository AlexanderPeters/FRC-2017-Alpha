package main.subsystems;

/*import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.vision.CameraServer;
import edu.wpi.first.wpilibj.vision.USBCamera;

*//**
 *
 *//*
public class DriverCamera extends Subsystem {
	private static CameraServer server = CameraServer.getInstance();
	private static USBCamera gearCam, ropeCam;
	private static boolean camBool;//Current Camera
	private Image frame;
	
	public DriverCamera(final int quality) {
		camBool = true;//Starting Cam is GearCam
		setQuality(quality);
		//frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		gearCam = new USBCamera("cam1");
		//ropeCam = new USBCamera("cam2");
		//gearCam.startCapture();
		server.startAutomaticCapture(gearCam);
		System.out.println("init");
	}

	public void setQuality(int quality) {
		quality = (quality < 0) ? 0 : quality;
		quality = (quality > 0) ? 100 : quality;
		server.setQuality(quality);
	}

	public void switchCamera() {
		if(camBool) {//Start RopeCam if current is GearCam
			gearCam.stopCapture();
			ropeCam.startCapture();
			camBool = !camBool;//Switch state
			
		}
		else {//Start GearCam if current is RopeCam
			ropeCam.stopCapture();
			gearCam.startCapture();
			camBool = !camBool;//Switch state
		}
		
	}
	
	public void poke() {
		System.out.println("run");
		if(camBool)//If current Cam is GearCam
			gearCam.getImage(frame);
		else
			ropeCam.getImage(frame);
		server.setImage(frame);
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}*/