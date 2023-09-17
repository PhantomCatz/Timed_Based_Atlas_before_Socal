package frc.Mechanisms.drivetrain;


import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.Odometry.CatzVision;
import frc.Mechanisms.Odometry.CatzRobotTracker;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.Robot.gameModeLED;

public class CatzDrivetrain {
    private static CatzDrivetrain instance = null;

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private static CatzSwerveModule[] swerveModules = new CatzSwerveModule[4];
    private static CatzVision limelightVision = CatzVision.getInstance();

    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;
    public final CatzSwerveModule RT_FRNT_MODULE;

    private final int LT_FRNT_DRIVE_ID = 1;
    private final int LT_BACK_DRIVE_ID = 3;
    private final int RT_FRNT_DRIVE_ID = 7;
    private final int RT_BACK_DRIVE_ID = 5;
    
    private final int LT_FRNT_STEER_ID = 2;
    private final int LT_BACK_STEER_ID = 4;
    private final int RT_FRNT_STEER_ID = 8;
    private final int RT_BACK_STEER_ID = 6;

    private final int LT_FRNT_ENC_PORT = 9;
    private final int LT_BACK_ENC_PORT = 6;
    private final int RT_FRNT_ENC_PORT = 8;
    private final int RT_BACK_ENC_PORT = 7;

    private double LT_FRNT_OFFSET = 0.0100; 
    private double LT_BACK_OFFSET = 0.0455;
    private double RT_BACK_OFFSET = 0.2572;
    private double RT_FRNT_OFFSET = 0.0284;

    private ChassisSpeeds chassisSpeeds;

    private CatzDrivetrain()
    {
        switch(CatzConstants.currentMode)
        {
        case REAL:
            gyroIO = new GyroIONavX();
        break;
        default:
            gyroIO = new GyroIONavX() {}; // TBD does gryo need sim class
        break;
        }

        LT_FRNT_MODULE = new CatzSwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET, 0);
        LT_BACK_MODULE = new CatzSwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET, 1);
        RT_FRNT_MODULE = new CatzSwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET, 2);
        RT_BACK_MODULE = new CatzSwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET, 3);

        swerveModules[0] = LT_FRNT_MODULE;
        swerveModules[1] = LT_BACK_MODULE;
        swerveModules[2] = RT_FRNT_MODULE;
        swerveModules[3] = RT_BACK_MODULE;


        resetMagEncs();
        //Reset Mag Enc after startup
        new Thread(() -> {
            try 
            {
                Thread.sleep(1000);
                zeroGyro();
            } 
            catch (Exception e) 
            {}
        }).start();
    }

    public void drivetrainPeriodic()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.periodic();
        }
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/gyroinputs ", gyroInputs);
    }


    /**********************************************************
     * 
     * CMD proc Swerve, determines logic of drivetrain every iteration of "main" looop
     * Called up in teleop periodic
     * 
     **********************************************************/
    public void cmdProcSwerve(double leftJoyX, double leftJoyY, double rightJoyX, boolean isAutoAlignCubeTrue, boolean isAutoAlignConeTrue)
    {


        if(Math.sqrt(Math.pow(leftJoyX,2) + Math.pow(leftJoyY,2)) < 0.1){
            leftJoyX = 0.0;
            leftJoyY = 0.0;
        }
        
        if(isAutoAlignCubeTrue) //Auto aim for cube nodes using apriltag
        {
            double drvPwrXCube;
            double drvPwrYCube;
            double turnPwrCube;
            if(limelightVision.disToTag() > 0.5){
                drvPwrXCube = 0.2;   
            }
            else if(limelightVision.disToTag() < 0.5){
                drvPwrXCube = -0.2;
            }
            else{
                drvPwrXCube = 0.0;
            }

            if(limelightVision.disLateralToTargetTag() > 0.1){
                drvPwrYCube = 0.2;
            }
            else if(limelightVision.disLateralToTargetTag() < -0.1){
                drvPwrYCube = -0.2;
            }
            else{
                drvPwrYCube = 0.0;
            }

            
            if(limelightVision.angleErrorFromTag() > 0.1) {
                turnPwrCube = 0.2;
            }
            else if(limelightVision.angleErrorFromTag() < -0.1){
                turnPwrCube = -0.2;
            }
            else{
                turnPwrCube = 0.0;
            }

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drvPwrXCube * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  drvPwrYCube * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  turnPwrCube * CatzConstants.DriveConstants.MAX_ANGSPEED, 
                                                                  getRotation2d());
        }
        else if(isAutoAlignConeTrue) //Auto aim for cone nodes using nodes + limelight
        {
            double drvPwrXCone;
            double drvPwrYCone;
            double turnPwrCone;
            if(limelightVision.getDistanceToTarget() > 0.5){
                drvPwrXCone = 0.2;   
            }
            else if(limelightVision.getDistanceToTarget() < 0.5){
                drvPwrXCone = -0.2;
            }
            else{
                drvPwrXCone = 0.0;
            }

            if(limelightVision.yErrorOffsetDeg > 0.1){
                drvPwrYCone = 0.2;
            }
            else if(limelightVision.yErrorOffsetDeg < -0.1){
                drvPwrYCone = -0.2;
            }
            else{
                drvPwrYCone = 0.0;
            }

            
            if(limelightVision.angleErrorFromTag() > 0.1) {
                turnPwrCone = 0.2;
            }
            else if(limelightVision.angleErrorFromTag() < -0.1){
                turnPwrCone = -0.2;
            }
            else{
                turnPwrCone = 0.0;
            }

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drvPwrXCone * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  drvPwrYCone * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  turnPwrCone * CatzConstants.DriveConstants.MAX_ANGSPEED, 
                                                                  getRotation2d());


        }
        else //in full teleop
        {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(leftJoyX * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  leftJoyY * CatzConstants.DriveConstants.MAX_SPEED, 
                                                                  rightJoyX * CatzConstants.DriveConstants.MAX_ANGSPEED, 
                                                                  getRotation2d());
        }


     
        SwerveModuleState[] moduleStates = CatzConstants.DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        setSwerveModuleStates(moduleStates);
    }//-End of Cmd Proc Swerve

    //inputs SwerveModule sates into each module depending on statenumeber
    public void setSwerveModuleStates(SwerveModuleState[] states)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, CatzConstants.DriveConstants.MAX_SPEED);

        Logger.getInstance().recordOutput("Drive/module states", states);
        Logger.getInstance().recordOutput("Drive/state speed pwr", states[0].speedMetersPerSecond);
        

        for(int i = 0; i < 4; i++)
        {
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    /*************************************************************
     * 
     * Swerve drive state misc methods
     * 
     ***********************************************************/
    public Rotation2d getRotation2d()
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getHeading() 
    {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];

        for(int i = 0; i < 4; i++)
        {
            moduleStates[i] = swerveModules[i].getModuleState();
        }

        return moduleStates;
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(int i = 0; i < 4; i++)
        {
            modulePositions[i] = swerveModules[i].getModulePosition();
        }

        return modulePositions;
    }

    public void printModulePositions()
    {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(int i = 0; i < 4; i++)
        {
            modulePositions[i] = swerveModules[i].getModulePosition();
            System.out.println(modulePositions[i].distanceMeters);
        }
    }

    /*************************************************************
     * 
     * Misc encoder/gyro methods
     * 
     ***********************************************************/
    private void resetMagEncs()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.resetMagEnc();
        }
    }

    public void resetDriveEncs()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.resetDriveEncs();
        }
    }

    public void initializeOffsets()
    {
        gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);

        for(CatzSwerveModule module : swerveModules)
        {
            module.initializeOffset();
        }
    }

    public void stopDriving()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.setPower(0.0);
            module.setSteeringPower(0.0);
        }
    }
    
    public double getGyroAngle()
    {
        return gyroInputs.gyroAngle;
    }

    public double getGyroYaw()
    {
        return gyroInputs.gyroYaw;
    }

    //zeros out the gryo to opposite what it's facing due to NavX instalation 180 degrees backwards
    public void zeroGyro()
    {
      gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);
    }


    /*************************************************************
    * 
    * Misc
    * 
    ***********************************************************/

    public void smartDashboardDriveTrain()
    {
        SmartDashboard.putNumber("NavX Gyro Angle", gyroInputs.gyroAngle);
        LT_FRNT_MODULE.smartDashboardModules();
        LT_BACK_MODULE.smartDashboardModules();
        RT_FRNT_MODULE.smartDashboardModules();
        RT_BACK_MODULE.smartDashboardModules();
    }    

    //returns itself for singleton implementation
    public static CatzDrivetrain getInstance()
    {
        if(instance == null)
        {
            instance = new CatzDrivetrain();
        }

        return instance;
    }

}
