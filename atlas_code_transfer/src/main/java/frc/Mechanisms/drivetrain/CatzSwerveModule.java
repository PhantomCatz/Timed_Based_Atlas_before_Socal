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

    public void setDesiredState(SwerveModuleState desiredState) //basically a function made solely for the purpose of following a trajectory. could be used for teleop though.
    {
        desiredState = SwerveModuleState.optimize(desiredState, getCurrentRotation()); //optimizes wheel rotation so that the furthest a wheel will ever rotate is 90 degrees.

        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO);
        io.setDrivePwrVelocityIO(velocity);

        //double targetAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (CatzConstants.DriveConstants.MAX_SPEED * 0.01)) ? getCurrentRotation().getDegrees() : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        double steerCommand = - steeringPID.calculate(getCurrentRotation().getDegrees(), desiredState.angle.getDegrees());  //negative sign due to xbox translation to numbers in wpilib
        steerCommand = Math.max(-1.0, Math.min(1.0, steerCommand));
        io.setSteerPwrIO(steerCommand);

        Logger.getInstance().recordOutput("Drive/targetError"       + Integer.toString(index), (getCurrentRotation().getDegrees() - desiredState.angle.getDegrees()));
        Logger.getInstance().recordOutput("Drive/targetPosition"    + Integer.toString(index), desiredState.angle.getDegrees());
        Logger.getInstance().recordOutput("Drive/currentRotation "  + Integer.toString(index), getCurrentRotation().getDegrees());
        Logger.getInstance().recordOutput("Drive/velocity "         + Integer.toString(index), velocity);
        Logger.getInstance().recordOutput("Drive/steeringpwr "      + Integer.toString(index), steerCommand);
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

    /**************************
     * 
     * @return
     * Current rotaiton object
     ***************************/
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
}
