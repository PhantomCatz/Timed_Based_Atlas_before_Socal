package frc.Autonomous;

import frc.Autonomous.Routines.AutonRoutineBase;

//runs a routine
public class AutonActionExecutor {
    private AutonRoutineBase routine = null;
    private Thread thread = null;

    private static AutonActionExecutor instance = null;

    // change the routine
    public void setAutonMode(AutonRoutineBase newRoutine) // this is run in the AutonRoutineSelector
    {
        this.routine = newRoutine;

        thread = new Thread(()->{
            if(routine != null)
            {
                routine.runAuton();
            }
        });
    }

    // start running the routine (important!!!!!!)
    public void start() //this is run in autonInit()
    {
        if(thread != null)
        {
            thread.start();
        }
        else
        {
            System.out.println("You forgot to set up the thread");
        }
    }

    // stops the current routine
    public void stop() //this is called in disabledInit()
    {
        if(routine != null)
        {
            routine.stop();
        }
        thread = null;
    }

    // returns this class
    public static AutonActionExecutor getInstance()
    {
        if(instance == null)
        {
            instance = new AutonActionExecutor();
        }
        return instance;
    }

    // resets this class
    public static void resetInstance()
    {
        if(instance != null)
        {
            instance = new AutonActionExecutor();
        }
    }
}
