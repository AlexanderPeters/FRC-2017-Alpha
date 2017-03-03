package lib;

/*import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import main.Constants;

public class UDPForVision implements Constants {
	public DatagramSocket serverSocket;
	public byte[] receiveData = new byte[1024];
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
		System.out.println("here!");
		serverSocket.receive(receivePacket);
		String sentence = new String(receivePacket.getData());
		String corrected = sentence.replaceAll("\u0000.*", "");
		String[] values = corrected.split(",");

		//Add an if statement to prevent out of bounds exceptions when values[x] is called
		System.out.println("X is " + Double.parseDouble(values[0]));
		System.out.println("Y is " + Double.parseDouble(values[1]));
		System.out.println("Distance is " + Double.parseDouble(values[2]));

	}
}
*/