/**
 * Team 3167, Father Judge High School
 */

package org.usfirst.frc.team3167.networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
 
 /**
  * Class used to receive datagram packets from the raspberry pi for target tracking.
  * The class uses UDP to receive packets holding data about the target being tracked by
  * the Pi's vision code.  To use, create an instance and call start(), then read the coordinates
  * it gets from the Pi using the PiVision object passed in the constructor
  */
public class PiVisionThread extends Thread
{
    private int port;				// Where we will receive the packets
    private DatagramSocket socket;  // Socket that will handle the packets
    private DatagramPacket packet;  // Packet received from Pi
    private byte[] buffer;			// Empty byte array used to clear out packet
    private byte[] receivedData;	// The newest data we got from the Pi
    private PiVision vision;		// The class that will interpret this raw data
    
    /**
     * Constructor
     * 
     * @param port		Local port we will be receiving packets at
     * @param vision	Object that will interpret the received byte stream
     */
    public PiVisionThread(int port, PiVision vision)
    {
    	this.vision = vision;
        this.port = port;
        try 
        {
			socket = new DatagramSocket(port);
		} 
        catch (SocketException e) 
        {
			e.printStackTrace();
		}
        buffer = new byte[2048];
        packet = new DatagramPacket(buffer, buffer.length);
        receivedData = new byte[2048];
    }
    
    @Override
    public void run()
    {
    	while(true)
    	{
    		receive();
    	}
    }
    
    /**
     * Waits on the port until a packet is received, then stores it in a class variable and passes
     * it to the PiVision object
     */
    private void receive()
    {
        try 
        {
			socket.receive(packet);
		}
        catch (IOException e)
        {
			e.printStackTrace();
		}
        receivedData = packet.getData();
        packet.setData(buffer);
        vision.setData(receivedData);
    }
    
}
