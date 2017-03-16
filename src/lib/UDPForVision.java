package lib;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import main.Constants;

public class UDPForVision implements Constants {
	public DatagramSocket serverSocket;
	public byte[] receiveData = new byte[1024];
	private String[] values;
	private double Time, CamNum, Range, Bearing = 0.0, Elevation;
	private boolean TargetFound = false;
	public DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);

	public UDPForVision() {
		try {
			serverSocket = new DatagramSocket(udpPort);
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public void poke() throws IOException {
		//System.out.println("here!");
		serverSocket.receive(receivePacket);
		String sentence = new String(receivePacket.getData());
		String corrected = sentence.replaceAll("\u0000.*", "");
		values = corrected.split(",");

		//if(values != null) {
			Time = Double.parseDouble(values[0]);
			CamNum = Double.parseDouble(values[1]);
			TargetFound = Boolean.parseBoolean(values[2]);
			Range = Double.parseDouble(values[3]);
			Bearing = Double.parseDouble(values[4]);
			Elevation = Double.parseDouble(values[5]);
			
			//Add an if statement to prevent out of bounds exceptions when values[x] is called
			System.out.println("Time is " + Time);
			System.out.println("CamNum is " + CamNum);
			System.out.println("TargetFound " + TargetFound);
			System.out.println("Range is " + Range);
			System.out.println("Bearing is " + Bearing);
			System.out.println("Elevation is " + Elevation);
		//}

	}
	
	public double getBearing() {
		return Bearing;
	}
	public boolean getTargetFound() {
		return TargetFound;
	}
}
