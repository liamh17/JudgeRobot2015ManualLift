/*******************************************************************************
* File:  StateMachineBase.java
* Date:  1/7/2014
* Auth:  Mark Macerato
* Desc:  Abstract base class for state machines
*******************************************************************************/

/**
 * Abstract base class for state machines 
 * 
 * @author Mark Macerato
 */
public abstract class StateMachineBase
{
    // Fields
    private byte state = -1; // -1 denotes the 'off' state, nothing has happened yet
    private byte nextState = 0; // the next state that the machine should advance to
     
    // Methods
     
    /**
     * Perform any tasks that the machine should be doing in its current
     * state.  Override this method with a switch statement based off of the
     * state, with each case containing the code to be executed if the robot
     * if in that state
     */
    protected abstract void executeStateTasks();
     
    /**
     * Perform any tasks which should occur only once when entering a given state. Override 
     * in the same manner as executeState(). Abstract method must be overriden in subclass.
     */
    protected abstract void enterState();
      
    /**
    * Perform any tasks which should occur only once when exiting a given state. Override 
    * in the same manner as executeState(). Abstract method must be overriden in subclass.
    */
    protected abstract void exitState();
      
    /**
     * Run the state machine
     */
    public final void execute()
    {
        if(state != nextState)
        {
          exitState();
          state = nextState;
          enterState();
        }
        executeStateTasks();
    }
      
    /**
     * Return the current state  
     */
    protected byte getState()
    {
        return state;
    }
      
     /**
      * Manually set the next state
      * 
      * @param nextState   The value assigned to the next state
      */
    protected final void setState(byte nextState)
    {
        this.nextState = nextState;
    }
}
