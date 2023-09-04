package frc.Autonomous.Routines;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.Autonomous.Actions.*;
import frc.Autonomous.Paths.Trajectories;

public class TestRoutine extends AutonRoutineBase{
    @Override
    protected void routine() {
        runAction(new TrajectoryFollowingAction(Trajectories.testTrajectoryStraight, Rotation2d.fromDegrees(0))); //follow the test trajectory while turning towards 180 degrees.
    }
}