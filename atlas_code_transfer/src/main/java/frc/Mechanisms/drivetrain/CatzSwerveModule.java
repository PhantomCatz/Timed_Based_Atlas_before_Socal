package frc.Mechanisms.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Utils.Conversions;
import frc.robot.CatzConstants;

public class CatzSwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged   inputs = new ModuleIOInputsAutoLogged();

    private final PIDController steeringPID;
    private final double kP = 0.005;
    private final double kI = 0.001;
    private final double kD = 0.0;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    private double wheelOffset;
    private int index;
    private int steerMotorID;

    private double currentAngle = 0.0;
    private double angleError = 0.0;
    private double flippedAngleError = 0.0;

    private double command;
    public boolean driveDirectionFlipped = false;

    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double wheelOffset,  int index)
    {

        MagEncPWMInput = new DigitalInput(encoderDIOChannel);
        magEnc = new DutyCycleEncoder(MagEncPWMInput);
        
        switch (CatzConstants.currentMode)
        {
            case REAL:
                    io = new ModuleIOReal(driveMotorID, steerMotorID, magEnc);
                break;
            case SIM :
                    io = null;//new ModuleIOSim(); TBD will we have swerve sim?
                break;
            default :
                    io = new ModuleIOReal(driveMotorID, steerMotorID, magEnc) {};
                break;
        }

        steeringPID = new PIDController(kP, kI, kD);


        this.wheelOffset = wheelOffset;
        this.steerMotorID = steerMotorID; //for smartdashboard

        this.index = index;
    }

    //Called up by drivetrain in Drivetrain perioidc
    public void periodic() 
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module " + Integer.toString(index), inputs);
    }

    /**
     * 
     * Turning logic
     **/
    public void setWheelAngle(double target, double gyroAngle)
    {
        currentAngle = ((magEnc.get() - wheelOffset) * 360.0) - gyroAngle;
        // find closest angle to target angle
        angleError = closestAngle(currentAngle, target);

        // find closest angle to target angle + 180
        flippedAngleError = closestAngle(currentAngle, target + 180.0);

        // if the closest angle to target is shorter
        if (Math.abs(angleError) <= Math.abs(flippedAngleError))
        {
            driveDirectionFlipped = false;
            command = steeringPID.calculate(currentAngle, currentAngle + angleError);
        }
        // if the closest angle to target + 180 is shorter
        else
        {
            driveDirectionFlipped = true;
            command = steeringPID.calculate(currentAngle, currentAngle + flippedAngleError);
        }

        command = -command / (180 * kP); //scale down command to a range of -1 to 1
        setSteerPower(command);
        
        Logger.getInstance().recordOutput("drivetrain/angle error" + Integer.toString(index), angleError);
        Logger.getInstance().recordOutput("drivetrain/steering cmd pwr" + Integer.toString(index), command);
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

    public double getEncValue()
    {
        return magEnc.get();
    }

    public double getDrvDistanceRaw()
    {
        return inputs.driveMtrSelectedSensorPosition;
    }

    public double getDrvDistance()
    {
        if(driveDirectionFlipped)
        {
            return inputs.driveMtrSelectedSensorPosition;
        }
        else
        {
            return -inputs.driveMtrSelectedSensorPosition;
        }
    }

    public void resetDrvDistance()
    {
        int i = 0;

        io.setDrvSensorPositionIO(0.0);
        while(Math.abs(inputs.driveMtrSelectedSensorPosition) > 1.0)
        {
            i++;
            if(i >= 3000)
            {
                resetDrvDistance();
            }
        }
    }

    public double getDrvVelocity()
    {
        return inputs.driveMtrSelectedSensorVelocity;
    }
    
    public double getAngle()
    {
        return inputs.magEncoderValue;//currentAngle
    }

    public double getWheelAngle()
    {
        return (inputs.magEncoderValue - wheelOffset) * 360.0;
    }

    public double getStrPwr()
    {
        return inputs.steerMotorAppliedOutput;
    }

    public double getError()
    {
        return angleError;
    }

    public double getFlipError()
    {
        return flippedAngleError;
    }

    //individual methods to manually command motors
    public void setDrivePower(double pwr)
    {
        if(driveDirectionFlipped == true)
        {
            pwr = -pwr;
        }
        io.setDrivePwrPercentIO(-pwr); //reversed to account for the flipped xbox drive cmds
    }

    public void setSteerPower(double speed)
    {
        io.setSteerPwrIO(speed);
    }

    //mag encoder methods
    public void resetMagEnc()
    {
        io.resetMagEncoderIO();
    }

    public void resetDriveEncs()
    {
        io.setDrvSensorPositionIO(0);
    }

    public void initializeOffset()
    {
        wheelOffset = inputs.magEncoderValue;
    }


    public double getDriveDistanceMeters()
    {
        return inputs.driveMtrSelectedSensorPosition / CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO * CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE / 2048.0;
    }

    public void smartDashboardModules()
    {
        SmartDashboard.putNumber(steerMotorID + " Mag Encoder", magEnc.get() );
    }


    public void setBrakeMode()
    {
        io.setSteerBrakeModeIO();

    }


    public void setCoastMode()
    {
        io.setSteerCoastModeIO();
        //DRIVE_MOTOR.setNeutralMode(NeutralMode.Coast); //REMOVE AFTER TESTING
    }

}
