package lib;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.List;

import main.Constants;

public class UDPForVision implements Constants {
	static int x, y, dist;
	public static int loop = 0;
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
		// @SuppressWarnings("resource")

		// byte[] sendData = new byte[1024];
		System.out.println("this");
		// while (true) {

		serverSocket.receive(receivePacket);
		String sentence = new String(receivePacket.getData());
		String corrected = sentence.replaceAll("\u0000.*", "");
		// System.out.println("RECEIVED: " + corrected);
		System.out.println(corrected);
		reader(corrected);
		loop = 0;
		print();
		// }

	}

	public static void reader(String str) {

		List<String> objects = new ArrayList<>();
		// System.out.println(str);
		String[] values = str.split(",");

		while (!values[loop].contains("Last")) {
			// System.out.println(loop);
			objects.add(values[loop]);
			loop++;
		}
		x = Integer.parseInt(objects.get(0));
		y = Integer.parseInt(objects.get(1));
		dist = Integer.parseInt(objects.get(2));
		objects = new ArrayList<>();

	}

	public static void print() {
		System.out.println("X is " + x);
		System.out.println("Y is " + y);
		System.out.println("Distance is " + dist);

	}

}
