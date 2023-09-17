package frc.Mechanisms.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO 
{
    @AutoLog
    public class ElevatorIOInputs
    {
        public double elevatorEncoderCnts = 0.0;
        public boolean isRevLimitSwitchClosed;
        public boolean isFwdLimitSwitchClosed;
        public double elevatorMotorPercentOutput = 0.0;
        public double elevatorCloseLoopError = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputsIO) {}

    public default void elevatorManualIO(double finalMtrPower) {}

    public default void elevatorConfig_kPIO(int slotID, double elevatorkP) {}

    public default void elevatorConfig_kIIO(int slotID, double elevatorkI) {}

    public default void elevatorConfig_kDIO(int slotID, double elevatorkD) {}

    public default void elevatorMtrSetPosIO(double setPosition) {}

    public default void configAllowableClosedloopErrorIO(int slotID, double closeloopErrorThreshold) {}

    public default void setSelectedSensorPositionIO(double setNewReadPosition) {}



    
    

}
