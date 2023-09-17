package frc.Mechanisms.Odometry;
    
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CatzVision {

    private static CatzVision instance = null;

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

    public boolean hasValidTarget;
    public boolean inScoringRange;

    public double xErrorOffsetDeg;    //Positive value = target is to the right, Negative value = target is to the left
    public double yErrorOffsetDeg;
    public double targetPresent;

    private final double LIMELIGHT_MOUNT_HEIGHT = 45.5;// TBD

    private final double LIMELIGHT_MOUNT_ANGLE = 31.0;// TBD what are these configs

    private final double HUB_TARGET_HEIGHT_TOP = 100.0;//TBD 

    private double angleToTargetDeg;
    private double angleToTargetRad;
    private double distanceToTargetInch;

    private CatzVision(int allianceColor)
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

    //uses limelight to for determining cone node dist
    public void coneNodeTracking()
    {
        targetPresent    = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        xErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        yErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);

        if (targetPresent == 1.0)
        {
            hasValidTarget = true;    
        }
        else
        {
            hasValidTarget  = false;
        }
        
        smartDashboardReflectiveTape();
    }


    
    /**********************************************************
    * Methods for other classes to determine cone/pose enttries
    * 
    **********************************************************/
    public boolean hasValidTarget()
    {
        return hasValidTarget;
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




    /**********************************************************
    * Methods for other classes to determine cone reflective tape dist
    * 
    **********************************************************/
    //math that determines cone node dist
    public double getDistanceToTarget()
    {
        getYErrorOffset();

        angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE + yErrorOffsetDeg;
        angleToTargetRad = angleToTargetDeg * (Math.PI / 180); //Convert angle from degrees to radians
        distanceToTargetInch = (HUB_TARGET_HEIGHT_TOP - LIMELIGHT_MOUNT_HEIGHT) / Math.tan(angleToTargetRad);

        return distanceToTargetInch;
    }

    //distance offset from to the right or left of the node
    public double getXErrorOffset()
    {
        xErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        return xErrorOffsetDeg - 3.28; //offset because turret aims too much to right
    }

    //distance offest from up or down of the node
    public double getYErrorOffset()
    {
        yErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        return yErrorOffsetDeg;
    }



    /****************************************
     * 
     * SmartDashboard methods for Apriltag and reflective tape
     * 
     **************************************/
    public void smartDashboardReflectiveTape()
    {
        SmartDashboard.putBoolean("Has Valid Target", hasValidTarget);
        SmartDashboard.putNumber("X Offset", xErrorOffsetDeg);
        SmartDashboard.putNumber("Y Offset", yErrorOffsetDeg);
        SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
    }

    public void smartDashboardAprilTag()
    {
        SmartDashboard.putNumber("botpos Px", botPose[POS_X_INDEX]);
        SmartDashboard.putNumber("botpos Py", botPose[POS_Y_INDEX]);
        SmartDashboard.putNumber("botpos Rz", botPose[ROT_Z_INDEX]);
    }

    public static CatzVision getInstance()
    {
        if(instance == null)
        {
            instance = new CatzVision(BLUE_ALLIANCE);
        }
        return instance;
    }
}
