/*******************************************************************************
 * File: ButterworthFilter.java
 * Auth: Mark Macerato
 * Desc: A second-order (two pole) low-pass Butterworth filter for digital 
 *       smoothing
 ******************************************************************************/
 package org.usfirst.frc.team3167.util;

 /**
  * A second-order (two pole) low-pass Butterworth filter for digital 
  * smoothing
  * 
  * @author Mark Macerato
  */
public class ButterworthFilter
{
    // Butterworth transfer function coefficients
    private final double a0, a1, a2;
    private final double b1, b2;
    
    // Variables storing a history of input and output values called for by the 
    // algorithm
    private double secondLastIn, lastIn, in;
    private double lastOut, out;
    
    /**
     * Constructor for the filter.  The cutoff frequency and sample rate must be
     * specified.
     * 
     * @param cutoff    Frequency above which filtering begins
     * @param samplingFreq  Rate at which data is acquired
     */ 
    public ButterworthFilter(double cutoff, double samplingFreq)
    {
        // Calculate the angular cutoff frequncy 
        double omega = (2 * Math.PI * cutoff)/samplingFreq;
        
        // Calculuate the warp cutoff frequency
        double warp = Math.tan(omega);
        
        // Compute Butterworth gain
        double gain = 1 / (1 + Math.sqrt(2)/warp + 2/(warp*warp));
        
        // Initialize data storage
        secondLastIn = 0.0;
        lastIn = 0.0;
        in = 0.0;
        lastOut = 0.0;
        out = 0.0;
        
        // Compute the H(z) coefficients
        a0 = gain;
        a1 = 2 * gain;
        a2 = gain;
        
        b1 = (2 - 2*2/(warp*warp)) * gain;
        b2 = (1 - Math.sqrt(2)/warp + 2/(warp*warp)) * gain;
    }
    
    /**
     * Apply the filter to a given input.  MUST be called at samplingFreq in the
     * main control loop
     * 
     * @param in    The input to the filter
     * @return      The filtered signal
     */
    public double filter(double in)
    {
        // Store the input, and move all values up in line
        secondLastIn = lastIn; lastIn = this.in; this.in = in;
        lastOut = out; 
        
        // Calculate the output of the filter using the algorithm
        out = a0*(this.in) + a1*lastIn + a2*secondLastIn - b1*out - b2*lastOut;
        
        return out;
    }
}  