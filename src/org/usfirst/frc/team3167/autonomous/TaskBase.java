/*******************************************************************************
* File: TaskBase.java
* Date: 1/10/14
* Auth: Mark Macerato
* Desc: Abstract template for a series of instructions 
*******************************************************************************/

/**
 * Class defining the template for a robot task using
 * a state machine
 * 
 * @author Mark Macerato
 */
public abstract class TaskBase extends StateMachineBase
{
    // Fields
    private String taskName;
    private boolean taskStarted;
    
    /**
     * Create a task with a chosen name
     * 
     * @param taskName  Name of the task
     */
    protected TaskBase(String taskName)
    {
        this.taskName = taskName;
        taskStarted = false;
    }
    
    /**
     * Carry out the instructions in the task, by running the state machine
     * that this class extends
     */
    protected final void executeTask()
    {
        taskStarted = true;
        execute();
    }
    
    /**
     * Details all instructions to be carried should this task be removed from
     * the task que.  Abstract method must be overriden in subclass.
     */ 
    protected abstract void onRemoval();
    
    /**
     * If the task has begun already, and it needs to be stopped, this method
     * will perform the needed instructions to abort the task
     */
    protected final void cleanupTask()
    {
        if(taskStarted) onRemoval();
    }
    
    /**
     * Whether or not the task has been completed yet, and the robot can move
     * on. Abstract method must be overriden in subclass.
     * 
     * @return  Whether the robot is done this task or not
     */ 
    protected abstract boolean isComplete();
    
    /**
     * Provides the name of the robots current state for the purpose of 
     * printing. Abstract method must be overriden in subclass.
     * 
     * @return  Name of the current state
     */ 
    protected abstract String getStateName();
    
    /**
     * Gets the name assigned to the task in the constructor, for printing 
     * purposes
     * 
     * @return  Name of the task
     */ 
    public final String getName()
    {
        return taskName;   
    }
    
    /**
     * Boolean indicating whether or not operator override is possible at this
     * time. Abstract method must be overriden in subclass.
     * 
     * @return  Whether the operator may intervene or not
     */ 
    protected abstract boolean okToDrive();
}
