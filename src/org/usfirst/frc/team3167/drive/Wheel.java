/*
 * Team 3167, Father Judge High School
 */
package org.usfirst.frc.team3167.drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.templates.util.PIDController;
import edu.wpi.first.wpilibj.templates.util.ButterworthFilter;

/**
 * Class representing a wheel on a drive train at a chosen position and 
 * orientation.  The assumptions are that a) the wheels make no angle with the  
 * ground, b) there are three or more wheels.  All wheels are attatched to an
 * actuator (a Jaguar speed controller), and may be mounted with an encoder.  If
 * not, then control in an open loop mode is allowed, if there is an encoder, 
 * closed loop control with a PIDController is used.
 * 
 * Additionally, a standard motor watchdog is used, that demands the speed 
 * controller be written to at a minimum frequency (NOTE: same mechanism is used
 * at speed controller level, so there is a 'double check' for safety)
 * 
 * COORDINATE SYSTEM DEFINITION:
 * 
 * X, Y, and Z are normal axes, defined by the Right Hand Rule; +Y is the direction
 * the robot is "looking" at.  Axis vector is parallel to wheel axis, and of
 * unit length.
 * 
 * @author Mark Macerato
 */
public class Wheel implements MotorSafety
{
    // Fields
    // Wheel position is (xPos, yPos) in chosen robot coordinate system.  You 
    // can choose any coordinate system you want, as long as it is the same
    // you use everywhere else.  Additionally, wheelAngle is the angle of the 
    // the axis of the wheel relative to the x-axis of the coordinate system.
    private final double xPos;
    private final double yPos;
    private final double wheelAngle;
    public final double radius;
    
    // Angle made by the rollers on the wheel rel to axis (if they exist, else 0 deg)
    private final double rollerAngle;
    
    // Motor driving the wheel, and the rate at which it should be actuated 
    // (only necessary for closed loop)
    private final Jaguar motor;
    private final double frequency;
    
    // Encoder attatched to wheel, and filter used to read it
    private Encoder encoder;
    private ButterworthFilter filter;
    private double wheelVelocity;
    private double lastPosition;
    
    // Control mode: open or closed
    private final byte control;
    private static final byte CLOSED = 0;
    private static final byte OPEN = 0;
    
    //PIDController that regulates the ouput in closed control mode
    private PIDController controller;
    
    // Kinematic variables
    private final double restSpeed = 0.01;
    private final double maxRotateSpeed;
    
    // Axis vector
    private double axisX;
    private double axisY;
    
    // Safety manager
    private MotorSafetyHelper safetyHelper;
    
    /**
     * Create a wheel without an encoder in an open control mode
     * 
     * @param xPos
     * @param yPos
     * @param wheelAngle
     * @param rollerAngle
     * @param motorChannel 
     * @param radius 
     * @param maxRotateSpeed 
     */
    public Wheel(double xPos, double yPos, double wheelAngle, double rollerAngle,
            int motorChannel, double radius, double maxRotateSpeed)
    {
        // Initialize geometric variables
        this.xPos = xPos;
        this.yPos = yPos;
        this.wheelAngle = wheelAngle;
        this.rollerAngle = rollerAngle;
        this.radius = radius;
        this.maxRotateSpeed = maxRotateSpeed;
        
        // Use wheel angle to computer unit vector on axis.  
        // IMPORTANT: This vector defines what a positive rotation is, by the
        // RHR
        axisX = Math.cos(Math.toRadians(wheelAngle));
        axisY = Math.sin(Math.toRadians(wheelAngle));
        
        // Create the driving motor
        motor = new Jaguar(1, motorChannel);
   
        // Frequency is not relevent to open control, so set it to 0
        frequency = 0.0;
        control = OPEN;
        
        handleMotorSafety();
    }
    
     /**
     * Create a wheel with an encoder
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
    public Wheel(double xPos, double yPos, double wheelAngle, double rollerAngle,
            int motorChannel, int encoderChannelA, int encoderChannelB, boolean encoderReversed,
            double pulsesPerRev, double encoderGearRatio, double radius,
            double maxRotateSpeed, double frequency, double Kp, double Ki, 
            double Kd, double maxIntegral, double maxOutput, double cutoff)
    {
        // Initialize geometric and kinematic variables
        this.xPos = xPos;
        this.yPos = yPos;
        this.wheelAngle = wheelAngle;
        this.rollerAngle = rollerAngle;
        this.radius = radius;
        this.maxRotateSpeed = maxRotateSpeed;
        this.frequency = frequency;
        
        // Use wheel angle to computer unit vector on axis.  
        // IMPORTANT: This vector defines what a positive rotation is, by the
        // RHR
        axisX = Math.cos(Math.toRadians(wheelAngle));
        axisY = Math.sin(Math.toRadians(wheelAngle));
        
        // Create the driving motor
        motor = new Jaguar(1, motorChannel);
        
        // Initialize encoder
        encoder = new Encoder(1, encoderChannelA, 1, encoderChannelB, encoderReversed);
        encoder.setDistancePerPulse(2 * Math.PI / pulsesPerRev / encoderGearRatio);
        encoder.reset();
        encoder.start();
        
        // Initialize filter
        filter = new ButterworthFilter(cutoff, frequency);
        // We're taking feedback
        control = CLOSED;
        
        // Initialize PID handling motor output
        controller = new PIDController(Kp, Ki, Kd, maxOutput, frequency, maxIntegral, cutoff);
        
        handleMotorSafety();
    }
    
    /**
     * Main control method.  Command the wheel an angular velocity, at the code
     * will try to attain it.  In closed control mode, the encoder and PIDController
     * will be used to achieve this omega.
     * 
     * @param   omega   The angular speed of the wheel [rad/s]
     */
    public void control(double omega)
    {
        // This signal will be given to the speed controller directly
        double motorSignal = 0.0;
        
        // We need different routines depending on the control mode we are 
        // operating under
        if(control == OPEN)
        {
            // An open control loop is never closed, e.g. it is "blind"
            // There is no feedback to the controller.  Use this if you have
            // no encoder
            wheelVelocity = omega;
            motorSignal = omega/maxRotateSpeed;
        }
        else if(control == CLOSED)
        {
            // Read the encoder
            double position = encoder.getDistance();
            // Use the last position and the current one to calculate the vel
            // Apply a filter to the encoder reading to smooth it
            wheelVelocity = filter.filter((position - lastPosition)*frequency);
            lastPosition = position;
            motorSignal = controller.updateOutput(omega, wheelVelocity);
        }
        
        // We've determined the value of the PWM signal, so send it
        motor.set(motorSignal);
        
        // Feed the watchdog
        safetyHelper.feed();
    }
        
    /**
     * Get the wheels x coordinate in robot coordinate system
     * 
     * @return  x coordinate 
     */
    public double getXPos()
    {
        return xPos;
    }
    
    /**
     * Get the wheels y coordinate in robot coordinate system
     * 
     * @return  y coordinate 
     */
    public double getYPos()
    {
        return yPos;
    }
    
    /**
     * Get the maximum wheel velocity
     * 
     * @return  The max rotate speed
     */
    public double getMaxRotation()
    {
        return maxRotateSpeed;
    }
    
    /**
     * Get the angle the bottom roller make with the ground
     * 
     * @return the angle the bottom rollers make with the ground
     */
    public double getRollerAngle()
    {
        return rollerAngle;
    }
    
    /**
     * Return the angles the wheels axis makes with the y axis
     * 
     * @return  angle the wheels makes with y 
     */
    public double getWheelAngle()
    {
        return wheelAngle;
    }
    
    /**
     * Get the x axis vector
     * 
     * @return x axis vector
     */
    public double getAxisX()
    {
        return axisX;
    }
    
    /**
     * Get the x axis vector
     * 
     * @return x axis vector
     */
    public double getAxisY()
    {
        return axisY;
    }
    
    /**
     * Get the radius of the wheel
     * 
     * @return  radius of the wheel 
     */
    public double getRadius()
    {
        return radius;
    }
    
    /**
     * Get the encoder object
     * 
     * @return  Encoder attatched to wheel 
     */
    public Encoder getEncoder()
    {
        return encoder;
    }
    
    /**
     * Get the actuators speed controller
     * 
     * @return  the Jaguar speed controller on the wheel 
     */
    public Jaguar getMotorController()
    {
        return motor;
    }
    
    /**
     * Get the wheels velocity
     * 
     * @return  the wheels velocity in rad/s
     */
    public double getWheelVelocity()
    {
        return wheelVelocity;
    }

    /**
     * Set the min time in between watchdog feedings that will kill the motor
     * 
     * @param d the expiration time
     */
    public void setExpiration(double d) 
    {
        safetyHelper.setExpiration(d);
    }

    /**
     * Get the min time that the watchdog must be feed, otherwise motor is 
     * killed
     * 
     * @return the expiration time
     */
    public double getExpiration() 
    {
        return safetyHelper.getExpiration();
    }

    /**
     * Check if PWM is still connected properly.  Passed to helper.
     * 
     * @return  is the PWN still alive 
     */
    public boolean isAlive() 
    {
        return safetyHelper.isAlive();
    }

    /**
     * Turn safety on or off, passed to helper
     * 
     * @param bln 
     */
    public void setSafetyEnabled(boolean bln)
    {
        safetyHelper.setSafetyEnabled(bln);
    }

    /**
     * Pass to safety helper, is the safety enabled
     * 
     * @return  true is safety is enabled 
     */
    public boolean isSafetyEnabled() 
    {
        return safetyHelper.isSafetyEnabled();
    }

    /**
     * Built in safety method must be overriden
     * 
     * @return a description of this PWM
     */
    public String getDescription() 
    {
        return "Jaguar connected to channel " + Integer.toString(motor.getChannel());
    }
    
    /**
     * Check if the wheel is slow enough to be considered at rest by our 
     * earlier definition
     * 
     * @return  true if wheel is basically at rest
     */
    public boolean isAtRest()
    {
        return wheelVelocity <= restSpeed;
    }
    
    /**
     * Bring the motor to a stop, by cutting the signal from the PWM.
     * 
     * NOTE: 'No Signal' != 'At Rest'.  The robot still needs to lose all of 
     * its momentum, so if you need it to stop immediately, take your own 
     * measures
     * 
     */
    public void stopMotor()
    {
        motor.set(0.0);
    }

    /**
     * Initialize the safety object
     */
    private void handleMotorSafety() 
    {
        safetyHelper = new MotorSafetyHelper(this);
        safetyHelper.setExpiration(MotorSafety.DEFAULT_SAFETY_EXPIRATION);
        safetyHelper.setSafetyEnabled(true);
    }
}
