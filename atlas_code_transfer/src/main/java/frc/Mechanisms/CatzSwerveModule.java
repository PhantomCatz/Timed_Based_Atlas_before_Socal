package frc.Mechanisms;

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
import frc.Utils.Conversions;
import frc.robot.CatzConstants;

public class CatzSwerveModule {
    private final CANSparkMax STEER_MOTOR;
    private final WPI_TalonFX DRIVE_MOTOR;

    private final PIDController steeringPID;
    private final double kP = 0.005;
    private final double kI = 0.0;
    private final double kD = 0.0;

    //private final int motorID; // uncomment when shuffleboard is added back

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    private double wheelOffset;

    private SupplyCurrentLimitConfiguration swerveModuleCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int     STEER_CURRENT_LIMIT_AMPS      = 30;

    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double wheelOffset)
    {
        STEER_MOTOR = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        DRIVE_MOTOR = new WPI_TalonFX(driveMotorID);

        swerveModuleCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        STEER_MOTOR.setSmartCurrentLimit(STEER_CURRENT_LIMIT_AMPS);
        DRIVE_MOTOR.configSupplyCurrentLimit(swerveModuleCurrentLimit);

        STEER_MOTOR.setIdleMode(IdleMode.kCoast);
        DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);

        DRIVE_MOTOR.config_kP(0, 0.1);
        DRIVE_MOTOR.config_kI(0, 0);
        DRIVE_MOTOR.config_kD(0, 0);

        steeringPID = new PIDController(kP, kI, kD);
        
        MagEncPWMInput = new DigitalInput(encoderDIOChannel);
        magEnc = new DutyCycleEncoder(MagEncPWMInput);

        this.wheelOffset = wheelOffset;
        //this.motorID = steerMotorID; //for smartdashboard
    }

    public void setDesiredState(SwerveModuleState desiredState) //basically a function made solely for the purpose of following a trajectory. could be used for teleop though.
    {
        desiredState = SwerveModuleState.optimize(desiredState, getCurrentRotation()); //optimizes wheel rotation so that the furthest a wheel will ever rotate is 90 degrees.

        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO);
        DRIVE_MOTOR.set(ControlMode.Velocity, velocity);

        double targetAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (CatzConstants.DriveConstants.MAX_SPEED * 0.01)) ? getCurrentRotation().getDegrees() : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        double steerCommand = - steeringPID.calculate(getCurrentRotation().getDegrees(), targetAngle);
        steerCommand = Math.max(-1.0, Math.min(1.0, steerCommand));
        STEER_MOTOR.set(steerCommand);
    }

    public void setPower(double power)
    {
        DRIVE_MOTOR.set(ControlMode.PercentOutput, power);
    }

    public void setSteeringPower(double speed)
    {
        STEER_MOTOR.set(speed);
    }

    public void resetMagEnc()
    {
        magEnc.reset();
    }

    public void resetDriveEncs()
    {
        DRIVE_MOTOR.setSelectedSensorPosition(0);
    }

    public void initializeOffset()
    {
        wheelOffset = magEnc.get();
    }

    private Rotation2d getCurrentRotation()
    {
        return Rotation2d.fromDegrees((magEnc.get() - wheelOffset)*360);
    }

    public SwerveModuleState getModuleState()
    {
        double velocity = Conversions.falconToMPS(DRIVE_MOTOR.getSelectedSensorVelocity(),CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO);
        
        return new SwerveModuleState(velocity, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition()
    {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }

    public double getDriveDistanceMeters()
    {
        return DRIVE_MOTOR.getSelectedSensorPosition() / CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO * CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE / 2048.0;
    }
}
