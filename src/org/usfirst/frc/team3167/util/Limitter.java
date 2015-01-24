package org.usfirst.frc.team3167.util;

/**
* A nested class that limits the value of a signal and its first derivative
* designed for velocity and acceleration
* 
* @author Mark Macerato
*/
public class Limitter
{
   private final double vMax, aMax;
   private double lastVel;
   private final double frequency;
   private boolean initialized;

   /**
    * Constructor
    * 
    * @param vMax
    * @param aMax
    * @param frequency
    */
   public Limitter(double vMax, double aMax, double frequency)
   {
       this.vMax = vMax;
       this.aMax = aMax;
       this.frequency = frequency;
       lastVel = 0.0;
       initialized = false;
   }

   /**
    * Take in a new velocity, and attenuate if needed
    * 
    * @param vel
     * @return 
    */
   public double process(double vel)
   {
       if(!initialized)
       {
           lastVel = vel;
           initialized = true;
           return vel;
       }
       // Compute acceleration
       double acc = (vel - lastVel)*frequency;

       // Limit acceleration
       if(acc > aMax)
       {
           acc = aMax;
       }
       else if(acc < -aMax)
       {
           acc = -aMax;
       }

       // Limit velocity
       if(vel + acc/frequency > vMax)
       {
           vel = vMax;
       }
       else if(vel + acc/frequency < -vMax)
       {
           vel = -vMax;
       }
       else
       {
           vel = vel + acc/frequency;
       }

       return vel;
   }
}