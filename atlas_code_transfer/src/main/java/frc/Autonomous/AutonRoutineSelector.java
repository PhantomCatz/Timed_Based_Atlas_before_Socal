package frc.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.Autonomous.Routines.AutonRoutineBase;
import frc.Autonomous.Routines.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

// allows the driver to use smartdashboard to choose the routine to be run during autonomous
public class AutonRoutineSelector {
    private static AutonRoutineSelector instance = null;

    // all the autonomous paths will be listed here (routine = sequence of actions = path)
    enum Routine
    {
        DO_NOTHING(new DoNothingRoutine()),
        TEST(new TestRoutine()),
        CURVE(new TestRoutineCurve()),
        CONNECT(new TestRoutineConnect()),
        SWERVE(new TestRoutineSwerve());

        public AutonRoutineBase routine;

        private Routine(AutonRoutineBase routine)
        {
            this.routine = routine;
        }
    }

    
    private Routine selectedRoutine = Routine.DO_NOTHING;

    private SendableChooser<Routine> routineChooser;

    private final AutonActionExecutor autonActionExecutor = Robot.autonExecutor;

    // sets up the sendable chooser with the paths stored in the enum
    private AutonRoutineSelector()
    {
        routineChooser = new SendableChooser<>();

        for(Routine routine : Routine.values()){
            routineChooser.addOption(routine.name(), routine);
        }

        SmartDashboard.putData(routineChooser);
    }

    // updates the routine to whatever is on the sendable chooser
    public void updateSelectedRoutine() //called in autonInit()
    {
        Routine routine = routineChooser.getSelected();

        if(routine == null)
        {
            selectedRoutine = Routine.DO_NOTHING;
        }
        else
        {
            selectedRoutine = routine;
        }

        autonActionExecutor.setAutonMode(selectedRoutine.routine);

        System.out.println("Selected routine: " + selectedRoutine.name());
    }

    // returns itself
    public static AutonRoutineSelector getInstance()
    {
        if(instance == null)
        {
            instance = new AutonRoutineSelector();
        }
        return instance;
    }
}
