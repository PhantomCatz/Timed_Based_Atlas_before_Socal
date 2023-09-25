package frc.Mechanisms.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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
    private final double kI = 0.0;
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
        //this.motorID = steerMotorID; //for smartdashboard

        this.index = index;
    }

    public void periodic() 
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module " + Integer.toString(index), inputs);
    }

    public void setDesiredState(SwerveModuleState desiredState) //basically a function made solely for the purpose of following a trajectory. could be used for teleop though.
    {
        desiredState = SwerveModuleState.optimize(desiredState, getCurrentRotation()); //optimizes wheel rotation so that the furthest a wheel will ever rotate is 90 degrees.

        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO);
        io.setDrivePwrVelocityIO(velocity);

        double targetAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (CatzConstants.DriveConstants.MAX_SPEED * 0.01)) ? getCurrentRotation().getDegrees() : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        double steerCommand = - steeringPID.calculate(getCurrentRotation().getDegrees(), targetAngle);
        steerCommand = Math.max(-1.0, Math.min(1.0, steerCommand));
        io.setSteerPwrIO(steerCommand);


        Logger.getInstance().recordOutput("Drive/targetError"       + Integer.toString(index), (getCurrentRotation().getDegrees() - desiredState.angle.getDegrees())); 
        Logger.getInstance().recordOutput("Drive/targetPosition"    + Integer.toString(index), desiredState.angle.getDegrees());
        Logger.getInstance().recordOutput("Drive/currentRotation"   + Integer.toString(index), getCurrentRotation().getDegrees());
        Logger.getInstance().recordOutput("Drive/velocity"          + Integer.toString(index), velocity);
        Logger.getInstance().recordOutput("Drive/steeringpwr"       + Integer.toString(index), steerCommand); 
        Logger.getInstance().recordOutput("Drive/distanceMeters"    + Integer.toString(index), getModulePosition().distanceMeters);
    }

    public void setPower(double power)
    {
        io.setDrivePwrPercentIO(power);
    }

    public void setSteeringPower(double speed)
    {
        io.setSteerPwrIO(speed);
    }

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

    private Rotation2d getCurrentRotation()
    {
        return Rotation2d.fromDegrees((inputs.magEncoderValue - wheelOffset)*360);
    }

    public SwerveModuleState getModuleState()
    {
        double velocity = Conversions.falconToMPS(inputs.driveMtrSelectedSensorVelocity , CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO);
        
        return new SwerveModuleState(velocity, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition()
    {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }

    public double getDriveDistanceMeters()
    {
        return inputs.driveMtrSelectedSensorPosition / CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO * CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE / 2048.0;
    }

    public void smartDashboardModules()
    {
        SmartDashboard.putNumber(steerMotorID + " Mag Encoder", magEnc.get() );
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
        io.setSteerPwrIO(command);
    }

    public void setSteerPower(double pwr)
    {
        io.setSteerPwrIO(pwr);
    }

    public void setDrivePower(double pwr)
    {
        if(driveDirectionFlipped == true)
        {
            pwr = -pwr;
        }
        io.setDrivePwrPercentIO(-pwr);
    }
}
