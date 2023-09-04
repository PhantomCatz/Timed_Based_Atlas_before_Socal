package frc.Autonomous.Routines;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.Autonomous.Actions.*;
import frc.Autonomous.Paths.Trajectories;

public class TestRoutineConnect extends AutonRoutineBase{
    @Override
    protected void routine() {
        runAction(new TrajectoryFollowingAction(Trajectories.testTrajectoryConnect1,Rotation2d.fromDegrees(0))); //Follow a trajectory, and follow another trajectory. 
        runAction(new TrajectoryFollowingAction(Trajectories.testTrajectoryConnect2,Rotation2d.fromDegrees(0)));
    }
}