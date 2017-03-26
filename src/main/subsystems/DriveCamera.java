package main.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class DriveCamera {
	CameraServer camServer;
	UsbCamera cam;
	public DriveCamera() {
		camServer = CameraServer.getInstance();
		cam = camServer.startAutomaticCapture();
		cam.setFPS(30);
		cam.setResolution(320, 240);
	}
}
