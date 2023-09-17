package frc.Mechanisms.elevator;

import frc.robot.CatzConstants;
import frc.robot.Robot;
//import frc.robot.Robot.mechMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;




public class ElevatorIOReal implements ElevatorIO
{
    private WPI_TalonFX elevatorMtr;

    private final int ELEVATOR_MC_ID = 10;
  
    private final double MANUAL_CONTROL_PWR_OFF = 0.0;
  
  
  
    //current limiting
    private SupplyCurrentLimitConfiguration elevatorCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;
  
  
  
    private final int SWITCH_CLOSED = 1;
  
    private boolean lowSwitchState  = false;
    private boolean highSwitchState = false;
  
  
  
    private final boolean LIMIT_SWITCH_IGNORED   = false;
    private final boolean LIMIT_SWITCH_MONITORED = true;  // limit switches will shut off the motor
  
  

  
    public ElevatorIOReal()
    {
        elevatorMtr = new WPI_TalonFX(ELEVATOR_MC_ID);

        elevatorMtr.configFactoryDefault();
    
        elevatorMtr.setNeutralMode(NeutralMode.Brake);
    
        elevatorMtr.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        elevatorMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        elevatorMtr.overrideLimitSwitchesEnable(LIMIT_SWITCH_MONITORED);
    
        elevatorCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);
    
        elevatorMtr.configSupplyCurrentLimit(elevatorCurrentLimit);
    
    
        elevatorMtr.config_kP(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_HIGH);
        elevatorMtr.config_kI(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_HIGH);
        elevatorMtr.config_kD(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_HIGH);
    
    
    
        elevatorMtr.config_IntegralZone(0, 2000.0);//TBD should go away once feet foward
    
        elevatorMtr.selectProfileSlot(0, 0);
    
        elevatorMtr.configAllowableClosedloopError(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);//make this constant and make values in inches
        
        elevatorMtr.set(ControlMode.PercentOutput, MANUAL_CONTROL_PWR_OFF);
    

    }

    //Input methods that are called to collect inputs every iteration in the "Main loop" called in CatzElevator.elevatorPerioidc
    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        inputs.elevatorEncoderCnts = elevatorMtr.getSelectedSensorPosition();
        inputs.isRevLimitSwitchClosed = (elevatorMtr.getSensorCollection().isRevLimitSwitchClosed() == SWITCH_CLOSED);
        inputs.isFwdLimitSwitchClosed = (elevatorMtr.getSensorCollection().isFwdLimitSwitchClosed() == SWITCH_CLOSED);
        inputs.elevatorMotorPercentOutput = elevatorMtr.getMotorOutputPercent();
        inputs.elevatorCloseLoopError = elevatorMtr.getClosedLoopError();
    }

    @Override
    public void elevatorManualIO(double setMtrPower) 
    {
        elevatorMtr.set(ControlMode.PercentOutput, setMtrPower);
    }

    @Override
    public void elevatorConfig_kPIO(int slotID, double elevatorkP) 
    {
        elevatorMtr.config_kP(0, elevatorkP);
    }

    @Override
    public void elevatorConfig_kIIO(int slotID, double elevatorkI) 
    {
        elevatorMtr.config_kI(0, elevatorkI);
    }

    @Override
    public void elevatorConfig_kDIO(int slotID, double elevatorkD) 
    {
        elevatorMtr.config_kD(0, elevatorkD);
    }

    @Override
    public void elevatorMtrSetPosIO( double setPositionEnc) 
    {
        elevatorMtr.set(ControlMode.Position, setPositionEnc,  DemandType.ArbitraryFeedForward, CatzConstants.ElevatorConstants.ELEVATOR_HOLDING_FF);
    }
    @Override
    public void configAllowableClosedloopErrorIO(int slotID, double closeloopErrorThreshold) 
    {
        elevatorMtr.configAllowableClosedloopError(slotID, closeloopErrorThreshold);
    }

    @Override
    public void setSelectedSensorPositionIO(double setNewReadPosition) 
    {
        elevatorMtr.setSelectedSensorPosition(setNewReadPosition);
    }   

 
}
