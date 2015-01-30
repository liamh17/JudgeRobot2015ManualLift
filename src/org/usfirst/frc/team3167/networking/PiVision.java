/**
 * Team 3167, Father Judge High School
 */

package org.usfirst.frc.team3167.networking;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
 
 /**
  * Class used to receive datagram packets from the raspberry pi for target tracking.
  * The class uses UDP to receive packets holding data about the target being tracked by
  * the Pi's vision code.
  */
public class PiVision
{
    private int port;
    private DatagramSocket socket;
    private DatagramPacket packet;
    private byte[] buffer;
    
    public PiVision(int port)
    {
        this.port = port;
        socket = new DatagramSocket(port);
        buffer = new byte[2048];
        packet = new DatagramPacket(buffer, buffer.length);
    }
    
    public void receive()
    {
        socket.receive();
    }
}
