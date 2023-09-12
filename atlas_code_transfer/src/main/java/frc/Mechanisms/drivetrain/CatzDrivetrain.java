package frc.Mechanisms.drivetrain;


import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.CatzConstants;

public class CatzDrivetrain {
    private static CatzDrivetrain instance = null;

    private static CatzSwerveModule[] swerveModules = new CatzSwerveModule[4];

    public final CatzSwerveModule RT_FRNT_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;
    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;

    private final int LT_FRNT_DRIVE_ID = 1;
    private final int LT_BACK_DRIVE_ID = 3;
    private final int RT_BACK_DRIVE_ID = 5;
    private final int RT_FRNT_DRIVE_ID = 7;
    
    private final int LT_FRNT_STEER_ID = 2;
    private final int LT_BACK_STEER_ID = 4;
    private final int RT_BACK_STEER_ID = 6;
    private final int RT_FRNT_STEER_ID = 8;

    private final int LT_FRNT_ENC_PORT = 9;
    private final int LT_BACK_ENC_PORT = 6;
    private final int RT_BACK_ENC_PORT = 7;
    private final int RT_FRNT_ENC_PORT = 8;

    private double LT_FRNT_OFFSET = 0.0091; 
    private double LT_BACK_OFFSET = 0.0466;
    private double RT_BACK_OFFSET = 0.2567;
    private double RT_FRNT_OFFSET = 0.0281;

    private AHRS navX;

    private CatzDrivetrain()
    {
        LT_FRNT_MODULE = new CatzSwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET, 0);
        LT_BACK_MODULE = new CatzSwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET, 1);
        RT_FRNT_MODULE = new CatzSwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET, 4);
        RT_BACK_MODULE = new CatzSwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET, 3);

        swerveModules[0] = LT_FRNT_MODULE;
        swerveModules[1] = LT_BACK_MODULE;
        swerveModules[2] = RT_FRNT_MODULE;
        swerveModules[3] = RT_BACK_MODULE;

        navX = new AHRS();
        navX.reset();

        resetMagEncs();
    }

    public void cmdProcSwerve(double leftJoyX, double leftJoyY, double rightJoyX, double pwrMode)
    {
        if(Math.sqrt(Math.pow(leftJoyX,2) + Math.pow(leftJoyY,2)) < 0.1){
            leftJoyX = 0.0;
            leftJoyY = 0.0;
        }

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(leftJoyX * CatzConstants.DriveConstants.MAX_SPEED, leftJoyY * CatzConstants.DriveConstants.MAX_SPEED, rightJoyX * CatzConstants.DriveConstants.MAX_ANGSPEED, getRotation2d());

        SwerveModuleState[] moduleStates = CatzConstants.DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        setSwerveModuleStates(moduleStates);

        Logger.getInstance().recordOutput("module states", moduleStates);

    }

    public void setSwerveModuleStates(SwerveModuleState[] states)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, CatzConstants.DriveConstants.MAX_SPEED);

        for(int i = 0; i < 4; i++)
        {
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    public Rotation2d getRotation2d()
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getHeading() 
    {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

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
        navX.setAngleAdjustment(-navX.getYaw());

        for(CatzSwerveModule module : swerveModules)
        {
            module.initializeOffset();
        }
    }

    public double getGyroAngle()
    {
        return navX.getAngle();
    }

    public void stopDriving(){
        for(CatzSwerveModule module : swerveModules)
        {
            module.setPower(0.0);
            module.setSteeringPower(0.0);
        }
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


    public static CatzDrivetrain getInstance()
    {
        if(instance == null)
        {
            instance = new CatzDrivetrain();
        }

        return instance;
    }
}
