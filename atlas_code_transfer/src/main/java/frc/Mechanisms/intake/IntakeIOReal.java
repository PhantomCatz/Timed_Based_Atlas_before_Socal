package frc.Mechanisms.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.CatzConstants;
import frc.robot.CatzConstants.IntakeConstants;

public class IntakeIOReal implements IntakeIO 
{

    //----------------------------------------------------------------------------------------------
    //
    //  Roller
    //
    //----------------------------------------------------------------------------------------------
    private WPI_TalonFX rollersMtr;

    private final int ROLLERS_MC_ID  = 30;

    private SupplyCurrentLimitConfiguration rollerCurrentLimit;

    private final int     CURRENT_LIMIT_AMPS_ROLLER            = 40;    
    private final int     CURRENT_LIMIT_TRIGGER_AMPS_ROLLER    = 40;

    //----------------------------------------------------------------------------------------------
    //
    //  Wrist
    //
    //----------------------------------------------------------------------------------------------
    private WPI_TalonFX wristMtr;

    private final int    WRIST_MC_ID   = 31;

    private final double WRIST_MAX_PWR = 0.3;

    //current limiting
    private SupplyCurrentLimitConfiguration wristCurrentLimit;

    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;
    

    //----------------------------------------------------------------------------------------------
    //  Wrist encoder & Position Values
    //----------------------------------------------------------------------------------------------
    private final int    WRIST_ENC_CAN_ID = 13; 





    private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
    private final double SOFT_LIMIT_REVERSE = -5850.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD



    public IntakeIOReal()
    {
        //----------------------------------------------------------------------------------------------
        //  Roller
        //----------------------------------------------------------------------------------------------
        rollersMtr = new WPI_TalonFX(ROLLERS_MC_ID);
        rollersMtr.configFactoryDefault();
        rollersMtr.setNeutralMode(NeutralMode.Brake);

        rollerCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS_ROLLER, 
                                                                                       CURRENT_LIMIT_TRIGGER_AMPS_ROLLER, 
                                                                                       CURRENT_LIMIT_TIMEOUT_SECONDS);

        rollersMtr.configSupplyCurrentLimit(rollerCurrentLimit);

        //----------------------------------------------------------------------------------------------
        //  Wrist
        //----------------------------------------------------------------------------------------------
        wristMtr = new WPI_TalonFX(WRIST_MC_ID);

        wristMtr.configFactoryDefault();
        /*
        * Code is functional in relative position...due to the gearings of the motor enc that causes many different positions of the intake to 
        *correlate with a potition on the motor encoder
        */    
      wristMtr.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
       // wristMtr.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360); 
        wristMtr.configIntegratedSensorOffset(0.0);
        
        wristMtr.setNeutralMode(NeutralMode.Brake);

        wristMtr.configForwardSoftLimitThreshold(SOFT_LIMIT_FORWARD);
        wristMtr.configReverseSoftLimitThreshold(SOFT_LIMIT_REVERSE);

        wristMtr.configForwardSoftLimitEnable(true);                  
        wristMtr.configReverseSoftLimitEnable(true);

        wristCurrentLimit  = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, 
                                                                 CURRENT_LIMIT_AMPS, 
                                                                 CURRENT_LIMIT_TRIGGER_AMPS, 
                                                                 CURRENT_LIMIT_TIMEOUT_SECONDS);        
        wristMtr.configSupplyCurrentLimit(wristCurrentLimit);

        wristMtr.configOpenloopRamp(0.5);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        inputs.wristPosEnc = wristMtr.getSelectedSensorPosition();
        inputs.wristTemp = wristMtr.getTemperature();
        inputs.wristAppliedPwr = wristMtr.getMotorOutputPercent();
    }

    @Override
    public void rollersOffIO() 
    {
        rollersMtr.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void rollersOnIO(double rollerPwrIO) 
    {
        rollersMtr.set(ControlMode.PercentOutput, rollerPwrIO);
    }

    @Override
    public void wristSetPercentOuputIO(double setIntakeMtrPwrIO) 
    {
        wristMtr.set(ControlMode.PercentOutput, setIntakeMtrPwrIO);
    }

    @Override
    public void intakeConfigureSoftLimitOverride(boolean enabled)
    {
        wristMtr.configForwardSoftLimitEnable(enabled);
        wristMtr.configReverseSoftLimitEnable(enabled);
    }
}
