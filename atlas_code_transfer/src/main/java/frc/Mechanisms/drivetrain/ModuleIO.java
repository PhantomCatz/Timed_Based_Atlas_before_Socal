package frc.Mechanisms.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO 
{
 @AutoLog
 public static class ModuleIOInputs
 {
    public double gyroAngle = 0.0;
    public double driveMtrSelectedSensorVelocity = 0.0;
    public double driveMtrSelectedSensorPosition = 0.0;
    public double magEncoderValue = 0.0;
    public double driveAppliedVolts = 0.0;
    public double steerAppliedVolts = 0.0;
    public double driveMtrOutputPercent = 0.0;
    public double magEncDistance = 0.0;
    public double steerMotorAppliedOutput = 0.0;

 }

 /** Updates the set of loggable inputs. */
 public default void updateInputs(ModuleIOInputs inputs) {}

 public default void setDrivePwrPercentIO(double drivePwrPercent) {}

 public default void setDrivePwrVelocityIO(double velocity) {}

 public default void setSteerPwrIO(double SteerPwr) {}

 public default void setSteerCoastModeIO() {}

 public default void setSteerBrakeModeIO() {}

 public default void setDrvSensorPositionIO(double sensorpos) {}

 public default void reverseDriveIO(boolean enable) {}

 public default void setDriveSimPwrIO(double volts) {}

 public default void setSteerSimPwrIO(double volts) {}

 public default void resetMagEncoderIO() {}

}
