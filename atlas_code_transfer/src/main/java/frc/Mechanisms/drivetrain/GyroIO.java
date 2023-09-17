package frc.Mechanisms.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO 
{
    @AutoLog
    public class GyroIOInputs {
    public double gyroAngle;
    public double gyroYaw;
    public double gyroRoll;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void resetNavXIO() {}

  public default void setAngleAdjustmentIO(double gyroYaw) {}
}
