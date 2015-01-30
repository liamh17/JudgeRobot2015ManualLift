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
  * the Pi's vision code.
  */
public class PiVisionThread extends Thread
{
    private int port;
    private DatagramSocket socket;
    private DatagramPacket packet;
    private byte[] buffer;
    private byte[] receivedData;
    
    public PiVisionThread(int port)
    {
        this.port = port;
        try 
        {
			socket = new DatagramSocket(port);
		} 
        catch (SocketException e) 
        {
			e.printStackTrace();
		}
        buffer = new byte[97];
        packet = new DatagramPacket(buffer, buffer.length);
        receivedData = new byte[97];
    }
    
    public void receive()
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
    }
}
