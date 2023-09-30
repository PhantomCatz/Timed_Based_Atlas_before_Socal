package frc.Mechanisms.arm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.elevator.CatzElevator;
import frc.Mechanisms.intake.CatzIntake;
import frc.robot.*;
import frc.robot.CatzConstants.ElevatorConstants;
import frc.robot.Robot.mechMode;

public class CatzArm
{
    private CatzElevator elevator = CatzElevator.getIntstance();
    private static CatzArm instance = null;
    private final ArmIO io;
    private final ArmIOInputsAutoLogged  inputs = new ArmIOInputsAutoLogged();


    private boolean extendSwitchState = false;

    private double targetPosition = -999.0;
    private double currentPosition = -999.0;
    private double positionError = -999.0; 

    private boolean armInPosition = false;
    private int numConsectSamples = 0;

    CatzLog data;

    boolean elevatorRaiseProcessEnabled = false;
    boolean isControlModePercent = false;

    double elevatorReadEnc;



    public CatzArm()
    {
        switch(CatzConstants.currentMode)
        {
            case REAL:
                io = new ArmIOReal();
                break;
            case SIM :
                io = null;
                break;
            default:
                io = new ArmIOReal() {};
                break;
        }
    }

    //returns itself for singleton implementation
    public static CatzArm getInstance()
    {
        if(instance == null)
        {
            instance = new CatzArm();
        }

        return instance;
    }

    //collect all arm inputs in robot periodic before any cmd proc units run
    public void armPeriodic()
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("arm", inputs);
        checkLimitSwitches();
        armPositionCheck();
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  cmdProcArm() called by telop periodic in the "main" loop
    *
    *----------------------------------------------------------------------------------------*/
    public void cmdProcArm(boolean armExtend, 
                           boolean armRetract,
                           int cmdUpdateState)
    {
        if(cmdUpdateState != Robot.COMMAND_STATE_NULL)
        {
            armInPosition = false;
            Robot.armControlMode = mechMode.AutoMode;
            switch(cmdUpdateState)
            {
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE :    
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE : 
                case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CUBE :
                case Robot.COMMAND_UPDATE_SCORE_LOW_CONE:
                case Robot.COMMAND_UPDATE_SCORE_LOW_CUBE:
                    setArmPos(CatzConstants.ArmConstants.POS_ENC_CNTS_PICKUP);
                    targetPosition = CatzConstants.ArmConstants.POS_ENC_CNTS_PICKUP;
                break;

                case Robot.COMMAND_UPDATE_SCORE_HIGH_CONE:
                case Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE:
                    elevatorRaiseProcess();
                    elevatorRaiseProcessEnabled = true;
                    targetPosition = CatzConstants.ArmConstants.POS_ENC_CNTS_EXTEND;
                break;

                case Robot.COMMAND_UPDATE_STOW           :
                case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CONE :
                case Robot.COMMAND_UPDATE_SCORE_MID_CUBE :
                case Robot.COMMAND_UPDATE_SCORE_MID_CONE :
                    setArmPos(CatzConstants.ArmConstants.POS_ENC_INCH_RETRACT);
                    targetPosition = CatzConstants.ArmConstants.POS_ENC_CNTS_RETRACT;
                break;
            }
            isControlModePercent = false;
            
        }

        //Logic for determining if we are in the elevator raise process
        if(elevatorRaiseProcessEnabled)
        {
            elevatorRaiseProcess();
        }

        if((cmdUpdateState != Robot.COMMAND_UPDATE_SCORE_HIGH_CONE ||
            cmdUpdateState != Robot.COMMAND_UPDATE_SCORE_HIGH_CONE) &&
            cmdUpdateState != Robot.COMMAND_STATE_NULL)
        {
            elevatorRaiseProcessEnabled = false;
        }



        //Manual Control of arm
        if(armExtend == true)
        {
            isControlModePercent = true;
            Robot.armControlMode = mechMode.ManualMode;
            setArmPwr(CatzConstants.ArmConstants.EXTEND_PWR);
        }
        else if(armRetract == true)
        {
            isControlModePercent = true;
            Robot.armControlMode = mechMode.ManualMode;
            setArmPwr(CatzConstants.ArmConstants.RETRACT_PWR);  
        }
        else if(isControlModePercent)
        {
            setArmPwr(CatzConstants.ArmConstants.MANUAL_CONTROL_PWR_OFF);
        }
        

        //Logging data
        Logger.getInstance().recordOutput("Arm/targetPosition", targetPosition);
        Logger.getInstance().recordOutput("Arm/positionError", positionError);
        Logger.getInstance().recordOutput("Arm/elevatorEncoderReading", elevator.isElevatorClearedThreshold());
        Logger.getInstance().recordOutput("Arm/isArmControlMode", inputs.controlMode == ControlMode.PercentOutput);

        if((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_INTAKE)) 
        {        
            data = new CatzLog(Robot.currentTime.get(), targetPosition, currentPosition, 
                                                        positionError,
                                                        -999.0, 
                                                        //armMtr.getMotorOutputPercent(),
                                                                    -999.0, -999.0, -999.0, -999.0, -999.0,
                                                                    -999.0, -999.0, -999.0, -999.0, -999.0,
                                                                    DataCollection.boolData);
                                    
            Robot.dataCollection.logData.add(data);
        }


    }   //End of cmdProcArm()


    /****
     * 
     *Start arm thread
     */
    private void armPositionCheck()
    {               
        //tracks the current position of the arm
        currentPosition = inputs.armMotorEncoder;
        positionError = currentPosition - targetPosition;
        if  ((Math.abs(positionError) <= CatzConstants.ArmConstants.ARM_POS_ERROR_THRESHOLD) && targetPosition != CatzConstants.ArmConstants.NO_TARGET_POSITION)
        {
            targetPosition = CatzConstants.ArmConstants.NO_TARGET_POSITION;
            numConsectSamples++;
                if(numConsectSamples >= 10)
                {   
                    armInPosition = true;
                }
        }

        else
        {
            numConsectSamples = 0;
        }

        Logger.getInstance().recordOutput("arm/threadtime", Logger.getInstance().getRealTimestamp());
    }

    
    //checks if elevator has cleared mid node before extending arm.
    private void elevatorRaiseProcess()
    {
        //extra if statment to ensure that when robot has cone in autonomous, the intake is in posiiton first
        if(DriverStation.isAutonomousEnabled() && Robot.selectedGamePiece == Robot.GP_CONE) 
        {
            if((elevatorReadEnc >= ElevatorConstants.ELEVATOR_ARM_ENCODER_THRESHOLD) && 
            CatzIntake.getInstance().isIntakeInPos())
            {
                io.setArmPosIO(CatzConstants.ArmConstants.POS_ENC_CNTS_EXTEND);
            }
        }
        else
        {
            if((elevatorReadEnc >= ElevatorConstants.ELEVATOR_ARM_ENCODER_THRESHOLD))
            {
                io.setArmPosIO(CatzConstants.ArmConstants.POS_ENC_CNTS_EXTEND);          
            }
        }

                
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  xxx()
    *
    *----------------------------------------------------------------------------------------*/
    public void checkLimitSwitches()
    {
        if(inputs.isRevLimitSwitchClosed)
        {
            io.setSelectedSensorPositionIO(CatzConstants.ArmConstants.POS_ENC_CNTS_RETRACT);
            extendSwitchState = true;
        }
        else
        {
            extendSwitchState = false;
        }
    }

    public void setArmPwr(double pwr)
    {        
        io.setArmPwrIO(pwr);
    }

    public void setArmPos(double position)
    {
        io.setArmPosIO(position);
    }

    public double getArmEncoder()
    {
        return inputs.armMotorEncoder;
    }

    public void smartDashboardARM()
    {

        SmartDashboard.putNumber("arm encoder position", inputs.armMotorEncoder);
    }
    public boolean isArmInPos()
    {
        return armInPosition;
    }

    public void setElevatorReadEnc(double recievedElevatorEnc)
    {
        elevatorReadEnc = 10 * recievedElevatorEnc;
    }
}
