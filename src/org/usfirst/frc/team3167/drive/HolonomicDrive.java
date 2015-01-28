/*
 * FIRST Team 3167, Father Judge High School
 */
package org.usfirst.frc.team3167.drive;

import edu.wpi.first.wpilibj.Gyro;

/**
 * A Holonomic drive that allows for robot oriented and field oriented control
 * schemes.  For the derivation of the equations used here, see both the comments
 * in this class, and the supporting document "Holonomic Drive Equations" by 
 * Kerry Loux.  
 * 
 * Coordinate System Definitions:
 * 
 * The field coordinate system is one that is centered on an arbitrary point on the field,
 * and contains x_f, y_f, and z_f axes that align with the dimensions of the field.  The robot operator
 * is looking in the positive Y direction, and his right is the positive X
 * direction.  The Z direction is positive upwards.  This coordinate system obeys
 * the right hand rule, and is independent of the robot's orientation.
 * 
 * The robot coordinate system is a set of coordinates x, y, and z, that are
 * always centered on the robot, and rotate to maintain the same orientation.
 * The positive direction of Y is the direction that the robot faces, the x
 * direction is positive to the robot's right, and z is positive upwards.
 * The right handle rule applies again.
 * 
 * Only use this class if the following assumptions are known to be true:
 * The robot has no motion in the z direction (i.e. no bumps), roller friction is
 * insignificant, and all wheel axes lie in the same horizontal plane.
 * 
 * To use: Construct an object of the class, with or without the Gyro, depending
 * on whether the driving should be robot-centric or field-centric.  Then, add 
 * wheels with or without encoders, and call the init() method to begin.  Use the
 * drive method to specify the robot's velocity in the field coordinate system.
 * 
 * @author Mark Macerato
 */
public class HolonomicDrive {
    
    private WheelCollection wheels;
    private final Gyro gyro;
    private final byte driveMode;
    
    private final double frequency;
    
    private static final byte FIELD_CENTRIC = 0;
    private static final byte ROBOT_CENTRIC = 1;
    
    private boolean initialized;

    /**
     * Constructor for a drive train with field centric gyro driving
     * 
     * @param gyro  The robot's gyroscope
     * @param frequency The frequency at which the control loop is closed
     */
    public HolonomicDrive(Gyro gyro, double frequency) 
    {
        this.gyro = gyro;
        this.frequency = frequency;
        wheels = new WheelCollection(0);
        
        driveMode = FIELD_CENTRIC;
        initialized = false;
    }
    
    /**
     * Constructor for a drive train with field centric gyro driving
     * 
     * @param frequency The frequency at which the control loop is closed
     */
    public HolonomicDrive(double frequency) 
    {
        // No gyro is needed here
        gyro = null;
        this.frequency = frequency;
        wheels = new WheelCollection(0);
        
        driveMode = ROBOT_CENTRIC;
    }
    
    /**
     * Start up the drive train, so that it may be driven.  This method will
     * make sure that at least three independent wheels have been added.
     * 
     * No more wheels can be added beyond this point, the drive train is now fixed
     */
    public void init()
    {
        initialized = true;
    }
    
    /**
     * Main driving method that uses a gyro for field oriented driving.  Specify
     * the robot's velocity relative to the field and its angular speed, and the program
     * will make it happen by setting the angular speeds of the wheels.
     * See the document mentioned in the class documentation for a derivation
     * by Kerry of the equations used here.  One additional step that has been 
     * added at the beginning is to rotate the field velocity to the robot 
     * coordinate system so that the driving is independent of the field.  
     * 
     * @param v_f   The velocity of the robot relative to the field
     * @param omega Robot's angular velocity about its center of rotation
     */
    public void drive(double[] v_f, double omega) throws IllegalStateException, ArithmeticException
    {
        // Check that the drive train has been initialized
        if(!initialized)
        {
            System.err.println("The drive train has not yet been "
                    + "initialized, so driving cannot yet occur.");
            throw new IllegalStateException("The drive train has not yet been "
                    + "initialized, so driving cannot yet occur.");
        }
        
        // Holds the angular velocity for all wheels
        double[] omegaCommand = new double[wheels.size()];
        
        // Calculate all wheel angular velocities first
        for(int i = 0; i < wheels.size(); i++)
        {
            Wheel wheel = wheels.get(i);
            double heading = gyro.getAngle();
            double position[] = {wheel.getXPos(), wheel.getYPos()};
            double axis[] = {wheel.getAxisX(), wheel.getAxisY()};
            double rollerAngle = wheel.getRollerAngle();
            double radius = wheel.getRadius();
        
            // Rotate the coordinate system of the field by the robot's heading, 
            // so that it now lines up with the robot.
            double[] v_r = rotateVector(v_f, heading, true);

            // Now, use this vector to calculate the velocity of the wheel.
            // Note: This is the cross product v cross p, where p points to
            // the wheel's center
            double[] v = {v_r[0] - omega*position[1], v_r[1] + omega*position[0]};

            // Find the unit vector that points along the wheel's direction of control.
            double[] w = rotateVector(axis, rollerAngle, false);

            // Find component of velocity along the axis of control
            double v_c = dotProduct(v, w);

            // Calculate the angular speed we need to drive the wheel at.
            // NOTE: Negative sign is to ensure that a positive angular velocity
            // drives the robot in the negative y direction, if it is parallel to it
            omegaCommand[i] = -1*(v_c/(radius * Math.sin(Math.toRadians(rollerAngle))));
        }
        
        // The calculation is done, so set the wheels to the right speed
        for(int i = 0; i < wheels.size(); i++)
        {
            Wheel wheel = wheels.get(i);
            wheel.control(omegaCommand[i]);
        }
    }
    
    /**
     * Add a wheel to the drive train with an open control loop, that is, with
     * no encoder feedback.
     * 
     * @param xPos
     * @param yPos
     * @param wheelAngle
     * @param rollerAngle
     * @param motorChannel
     * @param radius
     * @param maxRotateSpeed
     */
    public void addWheel(double xPos, double yPos, double wheelAngle,
            double rollerAngle, int motorChannel, double radius, 
            double maxRotateSpeed)
    {
        if(initialized)
        {
            throw new IllegalStateException("The drive train has been initialized, "
                    + "adding wheels is no longer permitted");
        }
        wheels.add(new Wheel(xPos, yPos, wheelAngle, rollerAngle, motorChannel, 
                radius, maxRotateSpeed));
    }
    
    /**
     * Add a wheel to the drive train with a closed control loop, that is, with
     * encoder feedback.  This wheel will be managed by a PIDController.
     * 
     * @param xPos
     * @param yPos
     * @param wheelAngle
     * @param rollerAngle
     * @param motorChannel
     * @param encoderChannelA
     * @param encoderChannelB
     * @param encoderReversed
     * @param pulsesPerRev
     * @param encoderGearRatio
     * @param radius
     * @param maxRotateSpeed
     * @param frequency
     * @param Kp
     * @param Ki
     * @param Kd
     * @param maxIntegral
     * @param maxOutput
     * @param cutoff
     */
    public void addWheel(double xPos, double yPos, double wheelAngle, double rollerAngle,
            int motorChannel, int encoderChannelA, int encoderChannelB, boolean encoderReversed,
            double pulsesPerRev, double encoderGearRatio, double radius,
            double maxRotateSpeed, double frequency, double Kp, double Ki, 
            double Kd, double maxIntegral, double maxOutput, double cutoff)
    {
        if(initialized)
        {
            throw new IllegalStateException("The drive train has been initialized, "
                    + "adding wheels is no longer permitted");
        }
        wheels.add(new Wheel(xPos, yPos, wheelAngle, rollerAngle, motorChannel, 
                encoderChannelA, encoderChannelB, encoderReversed, pulsesPerRev,
                encoderGearRatio, radius, maxRotateSpeed, frequency, Kp, Ki, Kd,
                maxIntegral, maxOutput, cutoff));
    }
    
    /**
     * Calculate the dot product of two vectors, assumed to be the same dimension
     */
    private double dotProduct(double[] vector1, double[] vector2)
    {
        // Variable to hold total number of dimensions
        int dimensions = 0;
        
        // Check that vectos have same dimension
        if(vector1.length != vector2.length)
        {
            throw new ArithmeticException("The vectors do not have the same length, "
                    + "vector one has length " + vector1.length + " and vector 2 "
                    + "has length " + vector2.length);
        }
        else
        {
            dimensions = vector1.length;
        }
        
        // Hold the sum so of the products of the components
        double sum = 0.0;
        
        // For every dimension, multiply the comopnents.  Add this to the sum.
        for(int i = 0; i < dimensions; i++)
        {
            sum += vector1[i]*vector2[i];
        }
        return sum;
    }
    
    /**
     * Rotate a vector counter-clockwise by some angle, either in an active or
     * passive sense.  Vector is assumed to be 2-dimensional, specified as
     * [V_x, V_y].
     * 
     * @param vector    Vector to be rotated
     * @param angle     Angle of rotation [degrees]
     * @param isPassive If the rotation is active or passive
     * 
     * @return  The new rotated vector 
     */
    private double[] rotateVector(double[] vector, double angle, boolean isPassive)
    {
        // Check to make sure this is actually a 2D vector
        if(vector.length != 2)
        {
            throw new ArithmeticException("This vector has a dimension of " + 
                    vector.length + " but, it should be 2");
        }
        
        // Convert angle to radians for trig functions, and find sin and cos
        angle = Math.toRadians(angle);
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        
        // Store vector components
        double x = vector[0];
        double y = vector[1];
        
        // Declare new components
        double xPrime = 0.0;
        double yPrime = 0.0;
        
        // If it's a passive rotation, do a passive rotation.  If not, do an 
        // active one
        if(isPassive)
        {
            xPrime = x*cos + y*sin;
            yPrime = -1*x*sin + y*cos;
        }
        else if(!isPassive)
        {
            xPrime = x*cos - y*sin;
            yPrime = 1*x*sin + y*cos;
        }
        
        // Store the new components to a vector and return in
        double[] vPrime = {xPrime, yPrime};
        
        return vPrime;
    }
}
