package frc.Mechanisms.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzDrivetrain 
{
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

    private final double LT_FRNT_OFFSET =  0.0168; //-0.0013; //MC ID 2
    private final double LT_BACK_OFFSET =  0.0432; //0.0498; //MC ID 4
    private final double RT_BACK_OFFSET =  0.2533; //0.2533; //MC ID 6
    private final double RT_FRNT_OFFSET =  0.0226; //0.0222; //MC ID 8

    private final double NOT_FIELD_RELATIVE = 0.0;

    private double steerAngle = 0.0;
    private double drivePower = 0.0;
    private double turnPower  = 0.0;
    private double gyroAngle  = 0.0;

    private boolean modifyDrvPwr = false;
    
    //Data collection
    public CatzLog data;

    public double dataJoystickAngle;
    public double dataJoystickPower;
    private double front;


    public CatzDrivetrain()
    {
        LT_FRNT_MODULE = new CatzSwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET);
        LT_BACK_MODULE = new CatzSwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET);
        RT_FRNT_MODULE = new CatzSwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET);
        RT_BACK_MODULE = new CatzSwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET);
 
        LT_FRNT_MODULE.resetMagEnc();
        LT_BACK_MODULE.resetMagEnc();
        RT_FRNT_MODULE.resetMagEnc();
        RT_BACK_MODULE.resetMagEnc();
    }

    public void cmdProcSwerve(double leftJoyX, double leftJoyY, double rightJoyX, double navXAngle, double pwrMode)
    {
        steerAngle = calcJoystickAngle(leftJoyX, leftJoyY);
        drivePower = calcJoystickPower(leftJoyX, leftJoyY);
        turnPower  = rightJoyX;
        
        gyroAngle  = navXAngle;

        
        //System.out.println(pwrMode);
        if(pwrMode > 0.9)
        {
           //System.out.println("---------");
            modifyDrvPwr = true;
        }
        else
        {
            modifyDrvPwr = false;
        }

        if(drivePower >= 0.1)
        {
            
            if(modifyDrvPwr == true)
            {
                drivePower = drivePower * 0.5;
                //System.out.println("MOD DRV");
            }
            

            if(Math.abs(turnPower) >= 0.1)
            {
                if(modifyDrvPwr == true)
                {
                    turnPower = turnPower * 0.5;
                    //System.out.println("MOD DRV");
                }
                translateTurn(steerAngle, drivePower, turnPower, gyroAngle);
                }
               else
            {
                drive(steerAngle, drivePower, gyroAngle);
            }

            dataCollection();
        }
        else if(Math.abs(turnPower) >= 0.1)
        {
            if(modifyDrvPwr == true)
            {
                turnPower = turnPower * 0.5;
                //System.out.println("MOD DRV");
            }
            
            rotateInPlace(turnPower);
            
            dataCollection();
        }
        else
        {
            setSteerPower(0.0);
            setDrivePower(0.0);
        }
    }

    public void drive(double joystickAngle, double joystickPower, double gyroAngle)
    {
        LT_FRNT_MODULE.setWheelAngle(joystickAngle, gyroAngle);
        LT_BACK_MODULE.setWheelAngle(joystickAngle, gyroAngle);
        RT_FRNT_MODULE.setWheelAngle(joystickAngle, gyroAngle);
        RT_BACK_MODULE.setWheelAngle(joystickAngle, gyroAngle);

        setDrivePower(joystickPower);

        dataJoystickAngle = joystickAngle;
        dataJoystickPower = joystickPower;
    }

    public void rotateInPlace(double pwr)
    {
        pwr *= 0.8; //reducing turn in place pwer
        LT_FRNT_MODULE.setWheelAngle(-45.0, NOT_FIELD_RELATIVE);
        LT_BACK_MODULE.setWheelAngle(45.0, NOT_FIELD_RELATIVE);
        RT_FRNT_MODULE.setWheelAngle(-135.0, NOT_FIELD_RELATIVE);
        RT_BACK_MODULE.setWheelAngle(135.0, NOT_FIELD_RELATIVE);

        LT_FRNT_MODULE.setDrivePower(pwr);
        LT_BACK_MODULE.setDrivePower(pwr);
        RT_FRNT_MODULE.setDrivePower(pwr);
        RT_BACK_MODULE.setDrivePower(pwr);
    }

    public void translateTurn(double joystickAngle, double translatePower, double turnPercentage, double gyroAngle)
    {
        //how far wheels turn determined by how far joystick is pushed (max of 45 degrees)
        double turnAngle = turnPercentage * -45.0;

        if(Math.abs(closestAngle(joystickAngle, 0.0 - gyroAngle)) <= 45.0)
        {
            // if directed towards front of robot
            front = 1.0;
            LT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
            RT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);

            LT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
        }
        else if(Math.abs(closestAngle(joystickAngle, 90.0 - gyroAngle)) < 45.0)
        {
            // if directed towards left of robot
            front = 2.0;
            LT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
            LT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);

            RT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
        }
        else if(Math.abs(closestAngle(joystickAngle, 180.0 - gyroAngle)) <= 45.0)
        {
            // if directed towards back of robot
            front = 3.0;
            LT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);

            LT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
            RT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
        }
        else if(Math.abs(closestAngle(joystickAngle, -90.0 - gyroAngle)) < 45.0)
        {
            // if directed towards right of robot
            front = 4.0;
            RT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);

            LT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
            LT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
        }

        LT_FRNT_MODULE.setDrivePower(translatePower);
        LT_BACK_MODULE.setDrivePower(translatePower);
        RT_FRNT_MODULE.setDrivePower(translatePower);
        RT_BACK_MODULE.setDrivePower(translatePower);
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

    public void dataCollection()
    {
        if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_SWERVE_STEERING)
        {
            data = new CatzLog(Robot.currentTime.get(), dataJoystickAngle,
                            LT_FRNT_MODULE.getAngle(), LT_FRNT_MODULE.getError(), LT_FRNT_MODULE.getFlipError(),
                            LT_BACK_MODULE.getAngle(), LT_BACK_MODULE.getError(), LT_BACK_MODULE.getFlipError(),
                            RT_FRNT_MODULE.getAngle(), RT_FRNT_MODULE.getError(), RT_FRNT_MODULE.getFlipError(),
                            RT_BACK_MODULE.getAngle(), RT_BACK_MODULE.getError(), RT_BACK_MODULE.getFlipError(), front, DataCollection.boolData);  
            Robot.dataCollection.logData.add(data);
        }
        else if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_SWERVE_DRIVING)
        {
            data = new CatzLog(Robot.currentTime.get(), dataJoystickAngle,
                            LT_FRNT_MODULE.getAngle(), LT_FRNT_MODULE.getDrvDistanceRaw(), LT_FRNT_MODULE.getDrvVelocity(),
                            LT_BACK_MODULE.getAngle(), LT_BACK_MODULE.getDrvDistanceRaw(), LT_BACK_MODULE.getDrvVelocity(),
                            RT_FRNT_MODULE.getAngle(), RT_FRNT_MODULE.getDrvDistanceRaw(), RT_FRNT_MODULE.getDrvVelocity(),
                            RT_BACK_MODULE.getAngle(), RT_BACK_MODULE.getDrvDistanceRaw(), RT_BACK_MODULE.getDrvVelocity(), LT_FRNT_MODULE.getError(), DataCollection.boolData);  
            Robot.dataCollection.logData.add(data);
        }
    }

    public void setSteerPower(double pwr)
    {
        LT_FRNT_MODULE.setSteerPower(pwr);
        LT_BACK_MODULE.setSteerPower(pwr);
        RT_FRNT_MODULE.setSteerPower(pwr);
        RT_BACK_MODULE.setSteerPower(pwr);
    }

    public void setDrivePower(double pwr)
    {
        LT_FRNT_MODULE.setDrivePower(pwr);
        LT_BACK_MODULE.setDrivePower(pwr);
        RT_FRNT_MODULE.setDrivePower(pwr);
        RT_BACK_MODULE.setDrivePower(pwr);
    }

    public void setBrakeMode()
    {
        LT_FRNT_MODULE.setBrakeMode();
        LT_BACK_MODULE.setBrakeMode();
        RT_FRNT_MODULE.setBrakeMode();
        RT_BACK_MODULE.setBrakeMode();
    }
    public void setCoastMode()
    {
        LT_FRNT_MODULE.setCoastMode();
        LT_BACK_MODULE.setCoastMode();
        RT_FRNT_MODULE.setCoastMode();
        RT_BACK_MODULE.setCoastMode();
    }

    
    public void smartDashboardDriveTrain()
    {
        LT_FRNT_MODULE.smartDashboardModules();
        LT_BACK_MODULE.smartDashboardModules();
        RT_FRNT_MODULE.smartDashboardModules();
        RT_BACK_MODULE.smartDashboardModules();
    }
    

    public void smartDashboardDriveTrain_DEBUG()
    {
        LT_FRNT_MODULE.smartDashboardModules_DEBUG();
        LT_BACK_MODULE.smartDashboardModules_DEBUG();
        RT_FRNT_MODULE.smartDashboardModules_DEBUG();
        RT_BACK_MODULE.smartDashboardModules_DEBUG();
        SmartDashboard.putNumber("Joystick", steerAngle);
    }


    public void zeroGyro()
    {
        Robot.navX.setAngleAdjustment(-Robot.navX.getYaw());
    }

    public double getGyroAngle()
    {
        return Robot.navX.getAngle();
    }

    public void autoDrive(double power)
    {
        LT_FRNT_MODULE.setWheelAngle(0, 0);
        LT_BACK_MODULE.setWheelAngle(0, 0);
        RT_FRNT_MODULE.setWheelAngle(0, 0);
        RT_BACK_MODULE.setWheelAngle(0, 0);

        setDrivePower(power);
    }


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

    public void lockWheels()
    {
        LT_FRNT_MODULE.setWheelAngle(-45.0, NOT_FIELD_RELATIVE);
        LT_BACK_MODULE.setWheelAngle(45.0, NOT_FIELD_RELATIVE);
        RT_FRNT_MODULE.setWheelAngle(-135.0, NOT_FIELD_RELATIVE);
        RT_BACK_MODULE.setWheelAngle(135.0, NOT_FIELD_RELATIVE);
    }

}
