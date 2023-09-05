package frc.Autonomous.Routines;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.Autonomous.Actions.*;
import frc.Autonomous.Paths.Trajectories;

public class TestRoutineCurve extends AutonRoutineBase{
    @Override
    protected void routine() {
        runAction(new TrajectoryFollowingAction(Trajectories.testTrajectoryCurve, Rotation2d.fromDegrees(0))); //Follow a curved trajectory
    }
}