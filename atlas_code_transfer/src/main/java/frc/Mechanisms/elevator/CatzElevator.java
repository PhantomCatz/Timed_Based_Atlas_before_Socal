package frc.Mechanisms.elevator;

import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.Robot.mechMode;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.Autonomous.CatzAutonomousPaths;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;



public class CatzElevator
{
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged  inputs = new ElevatorIOInputsAutoLogged();

    private final int ELEVATOR_MC_ID = 10;

    private final double MAX_MANUAL_SCALED_POWER = 0.7;

    private final double MANUAL_CONTROL_DEADBAND = 0.1;

    private final double MANUAL_CONTROL_PWR_OFF = 0.0;

    private boolean elevatorInManual = false;
    private double targetPositionEnc;

    private final double MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;
    

    //constants foir calc encoder to inch

    private final double FIRST_GEAR1 = 13;
    private final double FIRST_GEAR2 = 48;
    private final double FIRST_GEAR_RATIO = FIRST_GEAR2/FIRST_GEAR1;

    private final double HTD1 = 15;
    private final double HTD2 = 30;
    private final double HTD_RATIO = HTD2/HTD1;

    private final double SECOND_GEAR1 = 28;
    private final double SECOND_GEAR2 = 24;
    private final double SECOND_GEAR_RATIO = SECOND_GEAR2/SECOND_GEAR1;

    private final double FINAL_RATIO = FIRST_GEAR_RATIO*HTD_RATIO*SECOND_GEAR_RATIO;

    private final double CNTS_TO_REV = 2048/1;

    private final double SPROKET_DIAMETER = 1.751;
    private final double SPROKET_CIRCUMFERENCE = SPROKET_DIAMETER*Math.PI;

    private final double INCHES_TO_COUNTS_CONVERSTION_FACTOR = ((CNTS_TO_REV*FINAL_RATIO)/SPROKET_CIRCUMFERENCE);
    

    


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


    private final double POS_ENC_INCH_LOW  = 0.0;
    private final double POS_ENC_INCH_MID_CONE  = 39.616;
    private final double POS_ENC_INCH_MID_CUBE  = 26;
    private final double POS_ENC_INCH_HIGH = 47.187;

    private final double POS_ENC_CNTS_LOW  = POS_ENC_INCH_LOW * INCHES_TO_COUNTS_CONVERSTION_FACTOR;
    private final double POS_ENC_CNTS_MID_CONE  = POS_ENC_INCH_MID_CONE * INCHES_TO_COUNTS_CONVERSTION_FACTOR;//91000.0;// needs to be lower...too high
    // private final double POS_ENC_CNTS_MID_CUBE  = POS_ENC_INCH_MID_CUBE * INCHES_TO_COUNTS_CONVERSTION_FACTOR;
    private final double POS_ENC_CNTS_MID_CUBE = 50000.0;

    private final double POS_ENC_CNTS_HIGH = POS_ENC_INCH_HIGH * INCHES_TO_COUNTS_CONVERSTION_FACTOR;//111200.0;
    
    private final double ELEVATOR_KP_LOW = 0.03;
    private final double ELEVATOR_KI_LOW = 0.0002;
    private final double ELEVATOR_KD_LOW = 0.001;

    private final double ELEVATOR_KP_MID = 0.083;
    private final double ELEVATOR_KI_MID = 0.0002;
    private final double ELEVATOR_KD_MID = 0.0;

    private final double ELEVATOR_KP_HIGH = ELEVATOR_KP_MID;
    private final double ELEVATOR_KI_HIGH = ELEVATOR_KI_MID;
    private final double ELEVATOR_KD_HIGH = ELEVATOR_KD_MID;

    private final double ARM_ENCODER_THRESHOLD = 35000.0;

    private final double HOLDING_FEED_FORWARD = 0.044;


    private final double CLOSELOOP_ERROR_THRESHOLD_LOW = 50; 
   // private final double CLOSELOOP_ERROR_THRESHOLD_HIGH_MID = 300; //tbd remove
    private final double CLOSELOOP_ERROR_THRESHOLD_HIGH_MID = 225; 

    public CatzLog data;
    private Timer elevatorTime;

    private boolean armRetractingAndElevatorDescent = false;

    private double targetPosition = -999.0;
    private double currentPosition = -999.0;
    private double positionError = -999.0;

    private final double ELEVATOR_POS_ERROR_THRESHOLD = 1000.0; //0.424 inches

    private final double NO_TARGET_POSITION = -999999.0;

    private boolean elevatorInPosition = false;

    private int numConsectSamples = 0;

    public CatzElevator()
    {
        switch(CatzConstants.currentMode)
        {
            case REAL:
                io = new ElevatorIOReal();
                break;
            case SIM :
                io = null; //new ElevatorIOSim();
                break;
            default:
                io = new ElevatorIOReal() {};
                break;
        }
    }


    /*-----------------------------------------------------------------------------------------
    *  
    *  cmdProcElevator()
    *
    *----------------------------------------------------------------------------------------*/
    public void cmdProcElevator(double elevatorPwr, boolean manualMode, int cmdUpdateState)
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("elevator", inputs);
        
        checkLimitSwitches();

       elevatorPwr = -elevatorPwr; //reverses elevator pwr

        switch (cmdUpdateState)
        {
            case Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE:
            case Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE:
            case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CUBE:
            case Robot.COMMAND_UPDATE_SCORE_LOW_CUBE:  
            case Robot.COMMAND_UPDATE_SCORE_LOW_CONE:
            case Robot.COMMAND_UPDATE_STOW:

                armRetractingAndElevatorDescent = true;

            break;

            case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CONE:

                armRetractingAndElevatorDescent = false;
                elevatorSetToSinglePickup();

                break;

            case Robot.COMMAND_UPDATE_SCORE_MID_CONE:
         
                armRetractingAndElevatorDescent = false;
                elevatorSetToMidPosCone();
            break;

            case Robot.COMMAND_UPDATE_SCORE_MID_CUBE:
                armRetractingAndElevatorDescent = false;
                elevatorSetToMidPosCube();
            break;

            case Robot.COMMAND_UPDATE_PICKUP_DOUBLE_CONE:
            case Robot.COMMAND_UPDATE_PICKUP_DOUBLE_CUBE:
            case Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE:
            case Robot.COMMAND_UPDATE_SCORE_HIGH_CONE:
                armRetractingAndElevatorDescent = false;
                elevatorSetToHighPos();
            break;
            
        
            case Robot.COMMAND_STATE_NULL:
            break;
        }

        if(armRetractingAndElevatorDescent)
        {
            if (Robot.arm.getArmEncoder() <= ARM_ENCODER_THRESHOLD)
            { 
                elevatorSetToLowPos();
                armRetractingAndElevatorDescent = false;
            }
        }

        if(cmdUpdateState != Robot.COMMAND_STATE_NULL)
        {
           elevatorInManual = false;
           Robot.elevatorControlMode = mechMode.AutoMode;
        }

        if(manualMode)
        {
            elevatorInManual = true;
        }
        
        if(Math.abs(elevatorPwr) >= MANUAL_CONTROL_DEADBAND)
        {
            armRetractingAndElevatorDescent = false;
            if(elevatorInManual) // Full manual
            {
                Robot.elevatorControlMode = mechMode.ManualMode;

                elevatorManual(elevatorPwr);
            }
            else // Hold Position
            {
                Robot.elevatorControlMode = mechMode.ManualHoldMode;

                targetPositionEnc = inputs.elevatorEncoderCnts;
                targetPositionEnc = targetPositionEnc + (elevatorPwr * MANUAL_HOLD_STEP_SIZE);
                io.elevatorMtrSetPosIO(targetPositionEnc);
            }
        }
        else
        {
            if (elevatorInManual)
            {
                elevatorManual(0.0);
            }
        }

        
        currentPosition = inputs.elevatorEncoderCnts;
        positionError = currentPosition - targetPosition;

        if((Math.abs(positionError) <= ELEVATOR_POS_ERROR_THRESHOLD) && targetPosition != NO_TARGET_POSITION)
        {
            targetPosition = NO_TARGET_POSITION;
            numConsectSamples++;
                if(numConsectSamples >= 10) //-TBD this hasn;t been working for some mechanisms
                {   
                    elevatorInPosition = true;
                }
            }
        else
        {
            numConsectSamples = 0;
        }
        Logger.getInstance().recordOutput("targetEncManual", targetPositionEnc);


       if((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_ELEVATOR))
        {
          data = new CatzLog(elevatorTime.get(), -999.0,-999.0,//elevatorMtr.getSelectedSensorPosition(), elevatorMtr.getMotorOutputPercent(), 
                                                                  -999.0, 
                                                                  -999.0, 
                                                                  -999.0, 
                                                                  -999.0,
                                                                  -999.0, -999.0, -999.0, 
                                                                  -999.0, -999.0, -999.0, -999.0, -999.0,
                                                                  DataCollection.boolData);  
          Robot.dataCollection.logData.add(data);
       }

    }
    

    /*-----------------------------------------------------------------------------------------
    *  
    *  Misc
    *
    *----------------------------------------------------------------------------------------*/
    public void elevatorManual(double pwr)
    {
        double mtrPower;

        mtrPower = pwr * MAX_MANUAL_SCALED_POWER;

        io.elevatorMtrSetPosIO(mtrPower);
    }

    public void elevatorSetToLowPos()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_LOW);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_LOW);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_LOW);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_LOW);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW);
    }

    public void elevatorSetToMidPosCone()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_MID);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_MID);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_MID);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_MID_CONE);
    }

    public void elevatorSetToMidPosCube()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_MID);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_MID);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_MID);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_MID_CUBE);
    }

    public void elevatorSetToHighPos()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_HIGH);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_HIGH);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_HIGH);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_HIGH);
    }

    public void elevatorSetToSinglePickup()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_LOW);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_LOW);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_LOW);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_LOW);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_SINGLE_PICKUP);
    }
    


    public void checkLimitSwitches()
    {
        if(inputs.isRevLimitSwitchClosed)
        {
            io.setSelectedSensorPositionIO(POS_ENC_CNTS_LOW);
            lowSwitchState = true;
        }
        else
        {
            lowSwitchState = false;
        }

        if(inputs.isRevLimitSwitchClosed)
        {
            io.setSelectedSensorPositionIO(POS_ENC_CNTS_HIGH);
            highSwitchState = true;
        }
        else
        {
            highSwitchState = false;
        }
    }


    public double getElevatorEncoder()
    {
        return inputs.elevatorEncoderCnts;
    }

    public void smartDashboardElevator()
    {

        SmartDashboard.putBoolean("Low Limit Switch", lowSwitchState);
        SmartDashboard.putBoolean("High Limit Switch", highSwitchState);
    }

    public void smartDashboardElevator_DEBUG()
    {
        //SmartDashboard.putNumber("Elevator Enc Pos", elevatorMtr.getSelectedSensorPosition());
        //SmartDashboard.putNumber("Elev Closed Loop Error", elevatorMtr.getClosedLoopError());
    }

    public boolean isElevatorInPos()
    {
        return elevatorInPosition;
    }
      
}
