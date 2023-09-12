package frc.Mechanisms.Odometry;
    
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CatzAprilTag {

    private static CatzAprilTag instance = null;

    private final double METER_TO_INCH = 39.37;
    private final int REQUIRED_ARRAY_LENGTH = 6;
    private final int NUMBER_OF_LINEAR_DIMENSIONS = 3;
    private static final double DISX_TAG_TO_TAG = 570.32; //distance form this side apriltag to the other side apriltag

    private final int POS_X_INDEX = 0;
    private final int POS_Y_INDEX = 1;
    private final int ROT_Z_INDEX = 5;
    
    private static final int BLUE_ALLIANCE = 0;
    private static final int RED_ALLIANCE = 1;

    private double[] botPose = null;

    private int alliance;

    private CatzAprilTag(int allianceColor)
    {
        alliance = allianceColor;
    }

    //grab botpos from limelight load into this class, load INVALID_POSE_ENTY if no apriltag is seeing
    public void botPoseUpdate()
    {
        if(aprilTagInView() && NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(botPose).length == REQUIRED_ARRAY_LENGTH)
        {
            for(int i = 0; i < REQUIRED_ARRAY_LENGTH; i++)
            {
                if (i  < NUMBER_OF_LINEAR_DIMENSIONS && alliance == RED_ALLIANCE)
                {
                    botPose[i] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(botPose)[i]*METER_TO_INCH;
                }
                else if(i  < NUMBER_OF_LINEAR_DIMENSIONS)
                {
                    botPose[i] = -NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(botPose)[i]*METER_TO_INCH;
                }
                else if (i == ROT_Z_INDEX && alliance == RED_ALLIANCE)
                {
                    botPose[i] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(botPose)[i]-180.0;
                    botPose[i] = -Math.signum(botPose[i])*180.0 - (botPose[i] % 180.0);
                }
                else
                {
                    botPose[i] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(botPose)[i];
                }
            }
            Logger.getInstance().recordOutput("botpos Px", botPose[POS_X_INDEX]);
            Logger.getInstance().recordOutput("botpos Py", botPose[POS_Y_INDEX]);
            Logger.getInstance().recordOutput("botpos Rz", botPose[ROT_Z_INDEX]);
        }
        else
        {
            botPose = null;
        }
    }

    public boolean aprilTagInView()
    {
        return (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1) != -1);
    }

    public Pose2d getLimelightBotPose()
    {
        botPoseUpdate();
        return new Pose2d(botPose[POS_X_INDEX],botPose[POS_Y_INDEX],Rotation2d.fromDegrees(botPose[ROT_Z_INDEX]));
    }

    //return the distance from center of robot to apriltag
    public double disToTag()
    {
        return (DISX_TAG_TO_TAG/2-Math.abs(botPose[POS_X_INDEX]));
    }

    public double disLateralToTargetTag()
    {
        return botPose[POS_Y_INDEX];
    }

    public double angleErrorFromTag()
    {
        return (botPose[ROT_Z_INDEX]);
    }

    public static CatzAprilTag getInstance()
    {
        if(instance == null)
        {
            instance = new CatzAprilTag(BLUE_ALLIANCE);
        }
        return instance;
    }

    public void smartDashboardAprilTag()
    {
        SmartDashboard.putNumber("botpos Px", botPose[POS_X_INDEX]);
        SmartDashboard.putNumber("botpos Py", botPose[POS_Y_INDEX]);
        SmartDashboard.putNumber("botpos Rz", botPose[ROT_Z_INDEX]);
    }
}
