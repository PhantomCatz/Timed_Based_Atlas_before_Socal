package frc.Autonomous.Routines;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.Autonomous.Actions.*;
import frc.Autonomous.Paths.Trajectories;

public class TestRoutine extends AutonRoutineBase{
    @Override
    protected void routine() {
        runAction(new TrajectoryFollowingAction(Trajectories.testTrajectoryStraight, Rotation2d.fromDegrees(0))); //follow the test trajectory while turning towards 180 degrees.
    }
}