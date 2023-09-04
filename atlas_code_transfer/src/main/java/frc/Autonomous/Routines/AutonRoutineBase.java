package frc.Autonomous.Routines;

import frc.Autonomous.Actions.ActionBase;

// routines run any amount of actions sequentially (yuhyun is working on code for running actions in parallel)
public abstract class AutonRoutineBase {

    // the routine, which contains the calls to runAction()
    protected abstract void routine();

    private boolean isActive = false;
    private final int threadPeriodMS = 20;

    // starts the routine
    public void runAuton()
    {
        isActive = true;

        routine(); 
    }

    //runs an action as described by the comments in ActionBase.java
    public void runAction(ActionBase action)
    {
        action.init();

        while(isActive && !action.isFinished())
        {
            action.update();

            try
            {
                Thread.sleep(threadPeriodMS);
            }
            catch(InterruptedException e){}
        }

        action.end();
    }

    //ends the current action
    public final void stop()
    {
        isActive = false;
    }
}