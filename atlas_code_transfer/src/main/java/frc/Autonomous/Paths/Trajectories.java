package frc.Autonomous.Paths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class Trajectories {
    //all the trajectories will be created here
    public static final Trajectory testTrajectoryStraight = generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),  //starting point
        List.of(
            new Translation2d(3, 0)                   //any point inbetween start and end so a curve can be created.
        ),
        new Pose2d(6, 0, Rotation2d.fromDegrees(0)),//ending point
        1.5, 1.5
    );

    public static final Trajectory testTrajectoryCurve = generateTrajectory(
        new Pose2d(0,0,Rotation2d.fromDegrees(0)),

        List.of(
            new Translation2d(3,1)
        ),

        new Pose2d(5,0,Rotation2d.fromDegrees(0)),
        1.5, 1.5
    );

    public static final Trajectory testTrajectoryConnect1 = generateTrajectory(
        new Pose2d(0,0,Rotation2d.fromDegrees(0)),

        List.of(
            new Translation2d(1.5,0)
        ),

        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        1.5, 1.5
    );

    public static final Trajectory testTrajectoryConnect2 = generateTrajectory(
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),

        List.of(
            new Translation2d(3,1.5)
        ),

        new Pose2d(3,3,Rotation2d.fromDegrees(90)),
        1.5, 1.5
    );

    // generates trajectory
    private static Trajectory generateTrajectory(Pose2d startPos, List<Translation2d> waypoints, Pose2d endPos, double maxVel, double maxAccel)
    {
        return TrajectoryGenerator.generateTrajectory(startPos, waypoints, endPos, new TrajectoryConfig(maxVel, maxAccel));
    }
}
