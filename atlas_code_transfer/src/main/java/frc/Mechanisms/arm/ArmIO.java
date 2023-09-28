package frc.Mechanisms.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO 
{
    @AutoLog
    public class ArmIOInputs
    {
        public double armMotorEncoder;
        public boolean isRevLimitSwitchClosed;
        public boolean isArmControlModePercentOutput;
        public double armPercentOutput;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    
    public default void setSelectedSensorPositionIO(double encoderResetPos) {}

    public default void setArmPwrIO(double pwr) {}

    
    public default void armSetFullExtendPosIO() {}

    
    public default void armSetRetractPosIO() {}

    
    public default void armSetPickupPosIO() {}
}
