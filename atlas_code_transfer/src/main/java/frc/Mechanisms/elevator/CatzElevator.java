package frc.Mechanisms.elevator;

import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.Robot.mechMode;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.arm.CatzArm;



public class CatzElevator
{
    private static CatzArm arm = CatzArm.getInstance();


    private static CatzElevator instance = null;
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged  inputs = new ElevatorIOInputsAutoLogged();

    private boolean elevatorInManual = false;
    private double targetPositionEnc;

    private boolean lowSwitchState  = false;
    private boolean highSwitchState = false;

    public CatzLog data;
    private Timer elevatorTime;


    private double targetPosition = -999.0;
    private double currentPosition = -999.0;
    private double positionError = -999.0;

    private boolean elevatorInPosition = false;

    private int numConsectSamples = 0;

    private CatzElevator()
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
        elevatorPositionCheck();
    }

    //Called in Robot perioidc to collect all inputs at the START of every "main" loop cycle
    public void elevatorPeriodic()
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("elevator", inputs);
        checkLimitSwitches();
        elevatorPositionCheck();
    }
    
    /*-----------------------------------------------------------------------------------------
    *  
    *  cmdProcElevator()
    *
    *----------------------------------------------------------------------------------------*/
    public void cmdProcElevator(double elevatorPwr, boolean manualMode, int cmdUpdateState)
    {
       elevatorPwr = -elevatorPwr; //reverses elevator pwr due to how xbox translates stick movements to numbers between -1 and 1

        //Logic for when robot is in Automated Cmd State
        if(cmdUpdateState != Robot.COMMAND_STATE_NULL)
        {
            elevatorInManual = false;
            Robot.elevatorControlMode = mechMode.AutoMode;
            elevatorInPosition = false;

            switch (cmdUpdateState)
            {
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE:
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE:
                case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CUBE:
                case Robot.COMMAND_UPDATE_SCORE_LOW_CUBE:  
                case Robot.COMMAND_UPDATE_SCORE_LOW_CONE:
                case Robot.COMMAND_UPDATE_STOW:
                    armRetractProcess();
                break;

                case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CONE:
                    elevatorSetToSinglePickup();
                    break;

                case Robot.COMMAND_UPDATE_SCORE_MID_CONE:
                    elevatorSetToMidPosCone();
                break;

                case Robot.COMMAND_UPDATE_SCORE_MID_CUBE:
                    elevatorSetToMidPosCube();
                break;

                case Robot.COMMAND_UPDATE_PICKUP_DOUBLE_CONE:
                case Robot.COMMAND_UPDATE_PICKUP_DOUBLE_CUBE:
                case Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE:
                case Robot.COMMAND_UPDATE_SCORE_HIGH_CONE:
                    elevatorSetToHighPos();
                break;
            }
        }



        //FullManual Control Logic
        if(manualMode)
        {
            elevatorInManual = true;
        }
        else if(manualMode && elevatorInManual)
        {
            elevatorInManual = false;
        }
        
        //Manual Control of Elevator
        if(Math.abs(elevatorPwr) >= CatzConstants.ElevatorConstants.ELEVATOR_MANUAL_CONTROL_DEADBAND)
        {
            if(elevatorInManual) // Full manual
            {
                Robot.elevatorControlMode = mechMode.ManualMode;

                elevatorManual(elevatorPwr);
            }
            else // Hold Position
            {
                Robot.elevatorControlMode = mechMode.ManualHoldMode;

                targetPositionEnc = inputs.elevatorEncoderCnts;
                targetPositionEnc = targetPositionEnc + (elevatorPwr * CatzConstants.ElevatorConstants.ELEVATOR_MANUAL_HOLD_STEP_SIZE);
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




        //Logging elevator Outputs
        Logger.getInstance().recordOutput("Elevator/targetEncManual", targetPositionEnc);
        Logger.getInstance().recordOutput("Elevator/elevatorInPos", elevatorInPosition);
        Logger.getInstance().recordOutput("Elevator/armReadEncoder", arm.getArmEncoder());

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

    } //-End of CMd Proc Elevator

    private void elevatorPositionCheck()
    {
        //Logic for determining if Elevator has reached target Position
        currentPosition = inputs.elevatorEncoderCnts;
        positionError = currentPosition - targetPosition;

        if((Math.abs(positionError) <= CatzConstants.ElevatorConstants.ELEVATOR_POS_ERROR_THRESHOLD) && targetPosition != CatzConstants.ElevatorConstants.NO_TARGET_POSITION)
        {
            targetPosition = CatzConstants.ElevatorConstants.NO_TARGET_POSITION;
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
        Logger.getInstance().recordOutput("elevator/threadtime", Logger.getInstance().getRealTimestamp());
    }

    //logic used to determine if arm has cleared it's externion over mid nodes
    private void armRetractProcess()
    {
        Thread armRetractProcessThread = new Thread(() ->
        {
            boolean armRetractProcessEnabled = true;
            while(armRetractProcessEnabled)
            {
                if(arm.getArmEncoder() <= CatzConstants.ElevatorConstants.ELEVATOR_ARM_ENCODER_THRESHOLD)
                {
                    elevatorSetToLowPos();
                }
            }
        });
        armRetractProcessThread.start();
    }
    

    /*-----------------------------------------------------------------------------------------
    *  
    *  Manual
    *
    *----------------------------------------------------------------------------------------*/
    public void elevatorManual(double pwr)
    {
        double mtrPower;

        mtrPower = pwr * CatzConstants.ElevatorConstants.ELEVATOR_MAX_MANUAL_SCALED_POWER;

        io.elevatorMtrSetPosIO(mtrPower);
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  ELevator Set state cmds
    *
    *----------------------------------------------------------------------------------------*/
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
    

    /*-----------------------------------------------------------------------------------------
    *  
    *  ELevator liit switches logic to reset elevator positions
    *
    *----------------------------------------------------------------------------------------*/
    public void checkLimitSwitches()
    {
        if(inputs.isRevLimitSwitchClosed)
        {
            io.setSelectedSensorPositionIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW);
            lowSwitchState = true;
        }
        else
        {
            lowSwitchState = false;
        }

        if(inputs.isFwdLimitSwitchClosed)
        {
            io.setSelectedSensorPositionIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_HIGH);
            highSwitchState = true;
        }
        else
        {
            highSwitchState = false;
        }
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  misc
    *
    *----------------------------------------------------------------------------------------*/
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
        SmartDashboard.putNumber("Elevator Enc Pos", inputs.elevatorEncoderCnts);
        //SmartDashboard.putNumber("Elev Closed Loop Error", elevatorMtr.getClosedLoopError());
    }

    public boolean isElevatorInPos()
    {
        return elevatorInPosition;
    }

    //returns itself for singleton implementation
    public static CatzElevator getIntstance()
    {
        if(instance == null)
        {
            instance = new CatzElevator();
        }

        return instance;
    }
      
}
