/*
 * Team 3167, Father Judge High School
 */
package org.usfirst.frc.team3167.drive;

/**
 * Implementation of the standard ArrayList class that can be used for the
 * wheel objects, since that class is not available in FRC java
 * 
 * @author Mark Macerato
 */
public class WheelCollection 
{    
    // Size of the list, and an array that actually holds the objects
    private int size;
    private Wheel[] wheels;
    
    /**
     * Constructor
     * 
     * @param size size of the collection by default
     */
    public WheelCollection(int size)
    {
        this.size = size;
        wheels = new Wheel[size];
    }
    
    /**
     * Add a wheel to the collection.  If the collection is full, resize it
     * 
     * @param wheel     Wheel to be added
     */
    public void add(Wheel wheel)
    {
        if(wheels[size - 1] != null)
        {
            Wheel[] temp = new Wheel[size];
            System.arraycopy(wheels, 0, temp, 0, size);
            wheels = new Wheel[size + 1];
            System.arraycopy(temp, 0, wheels, 0, size);
            wheels[size] = wheel;
        }
        
        for(int i = 0; i < size; i++)
        {
            if(wheels[i] == null)
            {
                wheels[i] = wheel;
            }
        }
    }
    
    /**
     * Return the wheel at index i
     * 
     * @param i     Index of the wheel we want
     * @return      The wheel at index i
     */
    public Wheel get(int i)
    {
        return wheels[i];
    }
    
    /**
     * Get the size of the array list
     * 
     * @return  the size of the arraylist 
     */
    public int size()
    {
        return size;
    }
}
