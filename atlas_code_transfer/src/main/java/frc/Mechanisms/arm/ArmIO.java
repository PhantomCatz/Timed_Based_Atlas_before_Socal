package frc.Mechanisms.arm;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.ControlMode;

public interface ArmIO 
{
    @AutoLog
    public class ArmIOInputs
    {
        public double armMotorEncoder;
        public boolean isRevLimitSwitchClosed;
        public boolean isArmControlModePercentOutput;
        public double armPercentOutput;
        public ControlMode controlMode;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    
    public default void setSelectedSensorPositionIO(double encoderResetPos) {}

    public default void setArmPwrIO(double pwr) {}

    public default void setArmPosIO(double position) {}

    public default void setControlMode(ControlMode set) {}
}
