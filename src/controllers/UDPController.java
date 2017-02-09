package controllers;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import lib.Loop;
import main.Constants;

public class UDPController implements Loop, Constants{
	private int x, y, dist;
	private byte[] receiveData = new byte[1024];
	DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
	private byte[] sendData = new byte[1024];
	DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length);
	DatagramSocket serverSocket;

	@Override
	public void onStart() {
		try {
			serverSocket = new DatagramSocket(udpPort);
		} catch (SocketException e) {
			e.printStackTrace();
		}	
	}

	@Override
	public void onLoop() {
		try {
			serverSocket.receive(receivePacket);
		} catch (IOException e) {
			e.printStackTrace();
		}
		String sentence = new String(receivePacket.getData());
		String corrected = sentence.replaceAll("\u0000.*", "");
		System.out.println("RECEIVED: " + corrected);

		String[] values = corrected.split(",");
		
		x = Integer.parseInt(values[0]);
		y = Integer.parseInt(values[1]);
		dist = Integer.parseInt(values[2]);
		
		System.out.println("X is " + x);
		System.out.println("Y is " + y);
		System.out.println("Distance is " + dist);		
		
	}

	@Override
	public void onStop() {
		//no-op
	}
	

}
