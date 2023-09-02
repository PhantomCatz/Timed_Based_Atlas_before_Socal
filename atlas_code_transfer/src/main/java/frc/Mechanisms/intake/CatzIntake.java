package frc.Mechanisms.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.Robot.mechMode;
import frc.DataLogger.*;;

public class CatzIntake
{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final double ROLLERS_PWR_CUBE_IN = -0.8;   
    private final double ROLLERS_PWR_CONE_IN =  1.0; //TBD decide pwrs for all cube cone scoring rollers

    private final double ROLLERS_PWR_CUBE_OUT =  1.0;   
    private final double ROLLERS_PWR_CONE_OUT = -0.5;


    //----------------------------------------------------------------------------------------------
    //  Wrist encoder & Position Values
    //----------------------------------------------------------------------------------------------
    private final int    WRIST_ENC_CAN_ID = 13; 

    private final double WRIST_MAX_PWR = 0.3;


    private final double ENC_TO_INTAKE_GEAR_RATIO =  46.0/18.0;
    private final double WRIST_CNTS_PER_DEGREE    = 46.459; //(4096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;


    private final double MANUAL_HOLD_STEP_SIZE = 1.5;       

    //TBD - ADD comment for ref point
    private final double CENTER_OF_MASS_OFFSET_DEG     = 177.0; 
    private final double WRIST_ABS_ENC_OFFSET_DEG = 0.0; //Set to make stow pos equal to 0
    private final double WRIST_ABS_ENC_OFFSET = WRIST_ABS_ENC_OFFSET_DEG * WRIST_CNTS_PER_DEGREE;//-989.0; //Negative value means abs enc 0 is above intake angle 0   
    
    private final double STOW_ENC_POS               =  0.0 + WRIST_ABS_ENC_OFFSET_DEG;//4872.0 + WRIST_ABS_ENC_OFFSET; //3883
    private final double STOW_CUTOFF                =  -7.232 + WRIST_ABS_ENC_OFFSET_DEG;// + WRIST_ABS_ENC_OFFSET; //3670

    private final double INTAKE_CUBE_ENC_POS        =  -147.000 + WRIST_ABS_ENC_OFFSET_DEG;//1324.0 + WRIST_ABS_ENC_OFFSET;    //-335
    private final double INTAKE_CONE_ENC_POS_GROUND =  -184.524 + WRIST_ABS_ENC_OFFSET_DEG;//-306.0  + WRIST_ABS_ENC_OFFSET;  //-1295  
    private final double INTAKE_CONE_ENC_POS_SINGLE =  -116.400 + WRIST_ABS_ENC_OFFSET_DEG;//2089.0 + WRIST_ABS_ENC_OFFSET;  //1100 //TBD should we continue using inches or should we reply on counts

    private final double SCORE_CUBE_ENC_POS         =  -104.000 + WRIST_ABS_ENC_OFFSET_DEG;//1859.0 + WRIST_ABS_ENC_OFFSET;  //870     // Applies to low-mid-high

    private final double SCORE_CONE_HIGH_ENC_POS    =  -153.000 + WRIST_ABS_ENC_OFFSET_DEG;//289.0 + WRIST_ABS_ENC_OFFSET;  //-700
    private final double SCORE_CONE_MID_ENC_POS     = INTAKE_CONE_ENC_POS_GROUND; //TBD verify if its the same as high
    private final double SCORE_CONE_LOW_ENC_POS     = INTAKE_CONE_ENC_POS_GROUND; //TBD


    private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
    private final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD

    private final double GROSS_kP = 0.002472;//0.00009; 
    private final double GROSS_kI = 0.0;//000040;
    private final double GROSS_kD = 0.000291;//0.000007;

    private final double FINE_kP = 0.005234;//0.00009; 
    private final double FINE_kI = 0.0;//000008;
    private final double FINE_kD = 0.000291;//0.000007;
    
    private final double MAX_GRAVITY_FF = 0.055; //0.09

    
    private PIDController pid;
    
    private Boolean  pidEnable = false;

    private double   targetPositionDeg = STOW_ENC_POS;

    private double   targetPower = 0.0;
    private double   prevTargetPwr = 0.0;

    private double currentPosition = -999.0;
    private double positionError = -999.0; 
    private double prevCurrentPosition = -999.0;

    private boolean intakeInPosition = false;

    private final double INTAKE_POS_ERROR_THRESHOLD_DEG = 5.0;
    private final double PID_FINE_GROSS_THRESHOLD_DEG = 17.0;

    private double pidPower = 0.0;
    private double ffPower = 0.0;

    private int numConsectSamples = 0;



    //private double wristAngle;


    /*----------------------------------------------------------------------------------------------
    *
    *  Misc
    *
    *---------------------------------------------------------------------------------------------*/
    CatzLog data;

    private final double THREAD_PERIOD = 0.02;




    /*----------------------------------------------------------------------------------------------
    *
    *  CatzIntake()
    *
    *---------------------------------------------------------------------------------------------*/
    public CatzIntake()
    {
        switch(CatzConstants.currentMode)
        {
            case REAL:
                io = new IntakeIOReal();
                break;
            case SIM :
                io = null; //new IntakeIOSim();
                break;
            default:
                io = new IntakeIOReal() {};
                break;
        }
        
        pid = new PIDController(GROSS_kP, GROSS_kI, GROSS_kD);
       
    }



    /*----------------------------------------------------------------------------------------------
    *
    *  startIntakeThread()
    *
    *---------------------------------------------------------------------------------------------*/
    public void startIntakeThread()
    {
        Thread intakeThread = new Thread(() ->
        {
            while(true)
            {
                


                if((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_INTAKE)) 
                {        
                    data = new CatzLog(Robot.currentTime.get(), targetPositionDeg, currentPosition, 
                                                                targetPower, 
                                                                pidPower,
                                                                ffPower,
                                                                inputs.wristTargetPwr,
                                                                            -999.0, -999.0, -999.0, 
                                                                            -999.0, -999.0, -999.0, -999.0, -999.0,
                                                                            DataCollection.boolData);
                                            
                    Robot.dataCollection.logData.add(data);
                }


                Timer.delay(THREAD_PERIOD);    
            }   //End of while(true)
        });
        intakeThread.start();
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  cmdProcIntake()
    *
    *---------------------------------------------------------------------------------------------*/
    public void cmdProcIntake(double wristPwr, boolean rollersIn, boolean rollersOut, boolean manualMode, 
                                                                                      boolean softLimitOverride, 
                                                                                      int CmdStateUpdate, 
                                                                                      int gamePiece)
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);


        if(manualMode){                
            pidEnable = false;
            Robot.intakeControlMode = mechMode.ManualMode;
        }

        if(Math.abs(wristPwr) >= 0.1)//if we are apply wrist power manually
        {
            if (pidEnable == true)//check if in manual holding state
            {
                Robot.intakeControlMode = mechMode.ManualHoldMode;

                if(wristPwr > 0)
                {
                    targetPositionDeg = Math.min((targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_FORWARD);
                }
                else
                {
                    targetPositionDeg = Math.max((targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_REVERSE);
                }
                prevCurrentPosition = -prevCurrentPosition; //intialize for first time through thread loop, that checks stale position values
            }
            else //in full manual mode
            {
                targetPower = wristPwr * WRIST_MAX_PWR;  
                io.wristSetPercentOuputIO(targetPower);  
            }
        }
        else //Manual power is OFF
        {
            if(pidEnable == false)//if we are still in manual mode and want to hold intake in place
            {
                targetPower = 0.0;
                io.wristSetPercentOuputIO(targetPower);
            }
        }


        if(rollersIn)
        {
            if(gamePiece == Robot.GP_CUBE)
            {
                rollersInCube();
            } 
            else 
            {
                rollersInCone();
            }
        }
        else if(rollersOut)
        {
            if(gamePiece == Robot.GP_CUBE)
            {
                rollersOutCube();
            } 
            else 
            {
                rollersOutCone();
            }
        }
        else
        {
            rollersOff();
        }

        

        if(CmdStateUpdate != Robot.COMMAND_STATE_NULL)
        {
            pid.reset();
            pidEnable = true;
            intakeInPosition = false;
            Robot.intakeControlMode = mechMode.AutoMode;
            prevCurrentPosition = -prevCurrentPosition; //intialize for first time through thread loop, that checks stale position values

            switch(CmdStateUpdate)
            {
                case Robot.COMMAND_UPDATE_STOW :
                    targetPositionDeg = STOW_ENC_POS;
                    break;
                    
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE :
                    targetPositionDeg = INTAKE_CONE_ENC_POS_GROUND;
                    break;
                    
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE :
                    targetPositionDeg = INTAKE_CUBE_ENC_POS;
                    break;

                case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CONE :
                    targetPositionDeg = INTAKE_CONE_ENC_POS_SINGLE;
                    break;
                    
                case Robot.COMMAND_UPDATE_SCORE_LOW_CONE :
                    targetPositionDeg = SCORE_CONE_LOW_ENC_POS;
                    Timer.delay(0.1);//wait for arm to deploy -TBD is this a temp fix
                    break;
    
                case Robot.COMMAND_UPDATE_SCORE_LOW_CUBE :
                case Robot.COMMAND_UPDATE_SCORE_MID_CUBE :
                case Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE:
                    targetPositionDeg = SCORE_CUBE_ENC_POS;
                    break;
    
                case Robot.COMMAND_UPDATE_SCORE_MID_CONE :
                    targetPositionDeg = SCORE_CONE_MID_ENC_POS;
                    break;
                    
                case Robot.COMMAND_UPDATE_SCORE_HIGH_CONE :
                    targetPositionDeg = SCORE_CONE_HIGH_ENC_POS;
                    break;

                default:
                    pidEnable = false;
                    //TBD
                    break;
            }
        }

        if(softLimitOverride){
            io.intakeConfigureSoftLimitOverride(false);
        }
        else{
            io.intakeConfigureSoftLimitOverride(true);
        }

        if(pidEnable)   
        {
            //----------------------------------------------------------------------------------
            //  Chk if at final position
            //----------------------------------------------------------------------------------
            currentPosition = inputs.wristPosEnc / WRIST_CNTS_PER_DEGREE;

            positionError = currentPosition - targetPositionDeg;


            if  ((Math.abs(positionError) <= INTAKE_POS_ERROR_THRESHOLD_DEG))
            {
                numConsectSamples++;
                if(numConsectSamples >= 1)  //-TBD how can we raise the counter?
                {   
                    intakeInPosition = true;
                }
            }
            else
            {
                numConsectSamples = 0;
            }
            
            
            if(Math.abs(positionError) >= PID_FINE_GROSS_THRESHOLD_DEG)
            {
                pid.setP(GROSS_kP);
                pid.setI(GROSS_kI);
                pid.setD(GROSS_kD);
            }
            else if(Math.abs(positionError) < PID_FINE_GROSS_THRESHOLD_DEG)
            {
                pid.setP(FINE_kP);
                pid.setI(FINE_kI);
                pid.setD(FINE_kD);
            }

            pidPower = pid.calculate(currentPosition, targetPositionDeg);
            ffPower = calculateGravityFF();
            targetPower = pidPower + ffPower;

            //-------------------------------------------------------------
            //  checking if we did not get updated position value(Sampling Issue).
            //  If no change in position, this give invalid target power(kD issue). -TBD shouldn't d term zero out?
            //  Therefore, go with prev targetPower Value.
            //-------------------------------------------------------------------
            if(prevCurrentPosition == currentPosition)
            {
                targetPower = prevTargetPwr;
            }

            //----------------------------------------------------------------------------------
            //  If we are going to Stow Position & have passed the power cutoff angle, set
            //  power to 0, otherwise calculate new motor power based on position error and 
            //  current angle
            //----------------------------------------------------------------------------------
            if(targetPositionDeg == STOW_ENC_POS && currentPosition > STOW_CUTOFF)
            {
                targetPower = 0.0;
            }
            io.wristSetPercentOuputIO(targetPower);

            prevCurrentPosition = currentPosition;
            prevTargetPwr = targetPower;
        }
    }



    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - PID 
    *
    *---------------------------------------------------------------------------------------------*/
    public void resetPID(){
        pidEnable = false; //-TBD do we still need this
        pid.reset();
    }

    public void enablePID(boolean set){
        pidEnable = set;
    }

    public boolean getPIDEnabled(){
        return pidEnable;
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Rollers
    *
    *---------------------------------------------------------------------------------------------*/
    public void rollersOff()
    {
        io.rollersOffIO();
    }

    public void rollersInCube()
    {
        io.rollersOnIO(ROLLERS_PWR_CONE_IN);
    }

    public void rollersOutCube()
    {
        io.rollersOnIO(ROLLERS_PWR_CUBE_OUT);
    }

    public void rollersInCone()
    {
        io.rollersOnIO(ROLLERS_PWR_CONE_IN);
    }

    public void rollersOutCone()
    {
        io.rollersOnIO(ROLLERS_PWR_CONE_OUT);
    }
    

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Wrist
    *
    *---------------------------------------------------------------------------------------------*/
    public double calcWristAngle()
    {
        double wristAngle = ((inputs.wristPosEnc / WRIST_CNTS_PER_DEGREE) - WRIST_ABS_ENC_OFFSET_DEG);
        return wristAngle;
    }

    public double getWristPosition(){
        return inputs.wristPosEnc;
    }
    
    public double calculateGravityFF()
    {
        double radians = Math.toRadians(calcWristAngle() - CENTER_OF_MASS_OFFSET_DEG);
        double cosineScalar = Math.cos(radians);
        
        return MAX_GRAVITY_FF * cosineScalar;
    }

    public double intakeWristTemp()
    {
        return inputs.wristTemp;
    }
    
    public void shuffleboardIntake()
    {
    
    }

    public void smartdashboardIntakeDebug()
    {
        SmartDashboard.putNumber ("wrist ang",  calcWristAngle());
        SmartDashboard.putNumber ("GravityFF",       calculateGravityFF());
        SmartDashboard.putNumber ("IntakeClosedLoopError", pid.getPositionError());
        SmartDashboard.putNumber ("applied output",  inputs.wristTargetPwr);
        SmartDashboard.putBoolean("pid",             pidEnable);
        SmartDashboard.putNumber ("mtr abs", inputs.wristPosEnc);
    }

    public boolean isIntakeInPos()
    {
        return intakeInPosition;
    }


}