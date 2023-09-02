package frc.Mechanisms.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

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


    private final double ENC_TO_INTAKE_GEAR_RATIO =  46.0/18.0;
    private final double WRIST_CNTS_PER_DEGREE    = 46.459; //(4096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;


    private final double MANUAL_HOLD_STEP_SIZE = 1.5;       

    //TBD - ADD comment for ref point
    //Reference Point = wrist would be slight above "Parallel to the ground"
    private final double CENTER_OF_MASS_OFFSET_DEG     = 177.0; 
    private final double WRIST_ABS_ENC_OFFSET_DEG = 0.0; //Set to make stow pos equal to 0
    private final double WRIST_ABS_ENC_OFFSET = WRIST_ABS_ENC_OFFSET_DEG * WRIST_CNTS_PER_DEGREE;//-989.0; //Negative value means abs enc 0 is above intake angle 0   
    
    private final double STOW_ENC_POS               =  0.0 + WRIST_ABS_ENC_OFFSET_DEG;//4872.0 + WRIST_ABS_ENC_OFFSET; //3883
    private final double STOW_CUTOFF                =  -7.232 + WRIST_ABS_ENC_OFFSET_DEG;// + WRIST_ABS_ENC_OFFSET; //3670

    private final double INTAKE_CUBE_ENC_POS        =  -147.000 + WRIST_ABS_ENC_OFFSET_DEG;//1324.0 + WRIST_ABS_ENC_OFFSET;    //-335
    private final double INTAKE_PICKUP_CONE_ENC_POS_GROUND =  -184.524 + WRIST_ABS_ENC_OFFSET_DEG;//-306.0  + WRIST_ABS_ENC_OFFSET;  //-1295  
    private final double INTAKE_PICKUP_CONE_ENC_POS_SINGLE =  -116.400 + WRIST_ABS_ENC_OFFSET_DEG;//2089.0 + WRIST_ABS_ENC_OFFSET;  //1100

    private final double SCORE_CUBE_ENC_POS         =  -104.000 + WRIST_ABS_ENC_OFFSET_DEG;//1859.0 + WRIST_ABS_ENC_OFFSET;  //870     // Applies to low-mid-high

    private final double SCORE_CONE_HIGH_ENC_POS    =  -153.000 + WRIST_ABS_ENC_OFFSET_DEG;//289.0 + WRIST_ABS_ENC_OFFSET;  //-700
    private final double SCORE_CONE_MID_ENC_POS     = INTAKE_PICKUP_CONE_ENC_POS_GROUND; //TBD verify if its the same as high
    private final double SCORE_CONE_LOW_ENC_POS     = INTAKE_PICKUP_CONE_ENC_POS_GROUND; //TBD


    private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
    private final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD

    
    public Boolean  pidEnable = false;

    public double   targetPositionDeg = STOW_ENC_POS;

    private double   targetPower = 0.0;
    private double   prevTargetPwr = 0.0;

    private double currentPosition = -999.0;
    private double positionError = -999.0; 
    public double prevCurrentPosition = -999.0;

    private boolean intakeInPosition = false;

    private final double INTAKE_POS_ERROR_THRESHOLD_DEG = 5.0;
    private final double PID_FINE_GROSS_THRESHOLD_DEG = 17.0;

    private double pidPower = 0.0;
    private double ffPower = 0.0;

    private int numConsectSamples = 0;


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

        wristMtr.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        wristMtr.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
        wristMtr.configIntegratedSensorOffset(0.0);
        
        wristMtr.setNeutralMode(NeutralMode.Brake);

        wristMtr.configForwardSoftLimitThreshold(SOFT_LIMIT_FORWARD);
        wristMtr.configReverseSoftLimitThreshold(SOFT_LIMIT_REVERSE);

        wristMtr.configForwardSoftLimitEnable(true);                  
        wristMtr.configReverseSoftLimitEnable(true);

        wristCurrentLimit  = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, 
                                                                                       CURRENT_LIMIT_TRIGGER_AMPS, 
                                                                                       CURRENT_LIMIT_TIMEOUT_SECONDS);        
        wristMtr.configSupplyCurrentLimit(wristCurrentLimit);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        inputs.wristPosEnc = wristMtr.getSelectedSensorPosition();
        inputs.wristTemp = wristMtr.getTemperature();
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
    public void intakeManualHoldingIO(double targetHoldingPwrIO) 
    {
        wristMtr.set(ControlMode.PercentOutput, targetHoldingPwrIO);
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
