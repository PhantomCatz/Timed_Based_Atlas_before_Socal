package frc.Mechanisms.drivetrain;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.Odometry.CatzVision;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;

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

    private final double NOT_FIELD_RELATIVE = 0.0;

    private ChassisSpeeds chassisSpeeds;

    private CatzDrivetrain()
    {
        switch(CatzConstants.currentMode)
        {
        case REAL:
            gyroIO = new GyroIONavX();
        break;
        default:
            gyroIO = new GyroIONavX() {};
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

    //returns itself for singleton implementation
    public static CatzDrivetrain getInstance()
    {
        if(instance == null)
        {
            instance = new CatzDrivetrain();
        }

        return instance;
    }

    /**********************************************************
     * 
     * drivetrainPeriodic collects all inputs into the code before any logic is done
     *
     *  Called up in robot periodic
     * 
     **********************************************************/  
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
     public void cmdProcSwerve(double leftJoyX, double leftJoyY, double rightJoyX, boolean modifyDrvPwr)
     {
         double steerAngle = calcJoystickAngle(leftJoyX, leftJoyY);
         double drivePower = calcJoystickPower(leftJoyX, leftJoyY);
         double turnPower  = rightJoyX;
 
         
         if(drivePower >= 0.1)
         {
             
             if(modifyDrvPwr == true)
             {
                 drivePower = drivePower * 0.5;
             }
             
 
             if(Math.abs(turnPower) >= 0.1)
             {
                 translateTurn(steerAngle, drivePower, turnPower, gyroInputs.gyroAngle);
             }
             else
             {
                 drive(steerAngle, drivePower, gyroInputs.gyroAngle);
             }
 
         }
         else if(Math.abs(turnPower) >= 0.1)
         {
             rotateInPlace(turnPower);
            
         }
         else
         {
             setSteerPower(0.0);
             setDrivePower(0.0);
         }
        Logger.getInstance().recordOutput("steer Angle", steerAngle);
        Logger.getInstance().recordOutput("drivePwr", drivePower);
     }
 
     public void drive(double joystickAngle, double joystickPower, double gyroAngle)
     {
        for (CatzSwerveModule module : swerveModules)
        {
            module.setWheelAngle(joystickPower, gyroAngle);
        }
 
         setDrivePower(joystickPower);
 
     }
 
     public void rotateInPlace(double pwr)
     {
         pwr *= 0.8; //reducing turn in place pwer
         LT_FRNT_MODULE.setWheelAngle(-45.0, NOT_FIELD_RELATIVE);
         LT_BACK_MODULE.setWheelAngle(45.0, NOT_FIELD_RELATIVE);
         RT_FRNT_MODULE.setWheelAngle(-135.0, NOT_FIELD_RELATIVE);
         RT_BACK_MODULE.setWheelAngle(135.0, NOT_FIELD_RELATIVE);

         for(CatzSwerveModule module : swerveModules)
         {
             module.setDrivePower(pwr);
         }
     }
 
     public void translateTurn(double joystickAngle, double translatePower, double turnPercentage, double gyroAngle)
     {
         //how far wheels turn determined by how far joystick is pushed (max of 45 degrees)
         double turnAngle = turnPercentage * -45.0;
 
         if(Math.abs(closestAngle(joystickAngle, 0.0 - gyroAngle)) <= 45.0)
         {
             // if directed towards front of robot
             LT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
             RT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
 
             LT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
             RT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
         }
         else if(Math.abs(closestAngle(joystickAngle, 90.0 - gyroAngle)) < 45.0)
         {
             // if directed towards left of robot
             LT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
             LT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
 
             RT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
             RT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
         }
         else if(Math.abs(closestAngle(joystickAngle, 180.0 - gyroAngle)) <= 45.0)
         {
             // if directed towards back of robot
             LT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
             RT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
 
             LT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
             RT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
         }
         else if(Math.abs(closestAngle(joystickAngle, -90.0 - gyroAngle)) < 45.0)
         {
             // if directed towards right of robot
             RT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
             RT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
 
             LT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
             LT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
         }
 
         setDrivePower(translatePower);
 
     }

     public void setSteerPower(double pwr)
     {
        for(CatzSwerveModule module : swerveModules)
        {
            module.setSteerPower(pwr);
        }
     }
 
     public void setDrivePower(double pwr)
     {
        for(CatzSwerveModule module : swerveModules)
        {
            module.setDrivePower(pwr);
        }
     }
 
     public void setBrakeMode()
     {
        for(CatzSwerveModule module : swerveModules)
        {
            module.setBrakeMode();
        }
     }
     public void setCoastMode()
     {
        for(CatzSwerveModule module : swerveModules)
        {
            module.setCoastMode();
        }
     }

     public void lockWheels()
     {
         LT_FRNT_MODULE.setWheelAngle(-45.0, NOT_FIELD_RELATIVE);
         LT_BACK_MODULE.setWheelAngle(45.0, NOT_FIELD_RELATIVE);
         RT_FRNT_MODULE.setWheelAngle(-135.0, NOT_FIELD_RELATIVE);
         RT_BACK_MODULE.setWheelAngle(135.0, NOT_FIELD_RELATIVE);
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
            module.setDrivePower(0.0);
            module.setSteerPower(0.0);
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

        for(CatzSwerveModule module : swerveModules)
        {
            module.smartDashboardModules();
        }
    }    


    /**************************
     * 
     * Joystick calculations
     ***************************/
    public double calcJoystickAngle(double xJoy, double yJoy)
    {
        double angle = Math.atan(Math.abs(xJoy) / Math.abs(yJoy));
        angle *= (180 / Math.PI);

        if(yJoy <= 0)   //joystick pointed up
        {
            if(xJoy < 0)    //joystick pointed left
            {
              //no change
            }
            if(xJoy >= 0)   //joystick pointed right
            {
              angle = -angle;
            }
        }
        else    //joystick pointed down
        {
            if(xJoy < 0)    //joystick pointed left
            {
              angle = 180 - angle;
            }
            if(xJoy >= 0)   //joystick pointed right
            {
              angle = -180 + angle;
            }
        }
      return angle;
    }

    public double calcJoystickPower(double xJoy, double yJoy)
    {
      return (Math.sqrt(Math.pow(xJoy, 2) + Math.pow(yJoy, 2)));
    }

    public double closestAngle(double startAngle, double targetAngle)
    {
        // get direction
        double error = targetAngle % 360.0 - startAngle % 360.0;

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(error) > 180.0)
        {
            error = -(Math.signum(error) * 360.0) + error;
            //closest angle shouldn't be more than 180 degrees. If it is, use other direction
            if(error > 180.0)
            {
                error -= 360;
            }
        }

        return error;
    }
}
