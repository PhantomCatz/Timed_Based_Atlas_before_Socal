package frc.Mechanisms.arm;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import frc.robot.CatzConstants;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ArmIOReal implements ArmIO
{
    private WPI_TalonFX armMtr;

    private final int ARM_MC_ID = 20;
  
    private final double EXTEND_PWR  = 0.2;
    private final double RETRACT_PWR = -0.2;
  
    //Conversion factors
  
    //current limiting
    private SupplyCurrentLimitConfiguration armCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;
  

    private final boolean LIMIT_SWITCH_IGNORED = false;
    private final boolean LIMIT_SWITCH_MONITORED = true;

  
  
    private int SWITCH_CLOSED = 1;
  
    private final double ARM_KP = 0.15;
    private final double ARM_KI = 0.0001;
    private final double ARM_KD = 0.0;
  
    private final double ARM_CLOSELOOP_ERROR = 3000;
  
    private final double MANUAL_CONTROL_PWR_OFF = 0.0;
  
  
    public ArmIOReal()
    {
        armMtr = new WPI_TalonFX(ARM_MC_ID);

        armMtr.configFactoryDefault();
    
        armMtr.setNeutralMode(NeutralMode.Brake);
        armMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        armMtr.overrideLimitSwitchesEnable(LIMIT_SWITCH_MONITORED);
    
        armMtr.config_kP(0, ARM_KP);
        armMtr.config_kI(0, ARM_KI);
        armMtr.config_kD(0, ARM_KD);
    
        armCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);
    
        armMtr.configSupplyCurrentLimit(armCurrentLimit);
    
        armMtr.configAllowableClosedloopError(0, ARM_CLOSELOOP_ERROR);
    
        armMtr.set(ControlMode.PercentOutput, MANUAL_CONTROL_PWR_OFF);

        armMtr.configOpenloopRamp(1.5);
    
    }

    @Override
    public void updateInputs(ArmIOInputs inputs)
    {
        inputs.armMotorEncoder        = armMtr.getSelectedSensorPosition();
        inputs.isRevLimitSwitchClosed = (armMtr.getSensorCollection().isRevLimitSwitchClosed() == SWITCH_CLOSED);
        inputs.isArmControlModePercentOutput  = (armMtr.getControlMode() == ControlMode.PercentOutput);
        inputs.armPercentOutput       = armMtr.getMotorOutputPercent(); 
        inputs.controlMode                = armMtr.getControlMode();
    }
    @Override
    public void setSelectedSensorPositionIO(double encoderResetPos) 
    {
        armMtr.setSelectedSensorPosition(encoderResetPos);
    }
    
    @Override
    public void setArmPwrIO(double pwr)
    {
        armMtr.set(ControlMode.PercentOutput, pwr);
    }

    @Override
    public void setArmPosIO(double position) 
    {
        armMtr.set(ControlMode.Position, position);
    }



}
