package frc.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.drivetrain.CatzDrivetrain;
import frc.robot.*;

@SuppressWarnings("unused")
public class CatzAutonomous 
{

    private CatzDrivetrain drivetrain = CatzDrivetrain.getInstance();
    private final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio
    private final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio

    private final double DRV_S_GEAR_RATIO = SDS_L2_GEAR_RATIO; //set to which modules are being used

    private final double DRV_S_THREAD_PERIOD = 0.02;

    private final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV = 2048.0;

    private final double DRVTRAIN_WHEEL_DIAMETER             = 4.0;
    private final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);
    private final double DRVTRAIN_ENC_COUNTS_TO_INCH         = DRVTRAIN_WHEEL_CIRCUMFERENCE / TALONFX_INTEGRATED_ENC_CNTS_PER_REV / DRV_S_GEAR_RATIO;

    //drive straight variables
    //drive power
    private final double DRV_S_KP = 0.02; //TBD // decel distance in inches = DRV_S_MAX_POWER/DRV_S_KP = 35 inch
    private final double DRV_BACK_KP = 0.035;

    private final double DRV_S_THRESHOLD_INCH = 0.5;
    private final double DRV_S_MIN_POWER      = 0.1;
    private final double DRV_S_MAX_POWER      = 0.7;
    private final double DRV_S_MAX_POWER_CHARGE_STATION = 0.5;

    private final double DRV_S_MIN_POWER_ROLL_BACK = 0.15;
    private final double DRV_S_MAX_POWER_ROLL_BACK = 0.7;

    //turn power
    private final double DRV_S_ERROR_GAIN    = 0.032;
    private final double DRV_S_RATE_GAIN     = 0.003;

    private final double DRV_S_ERROR_GAIN_ON_STATION = 0.048;//increased by 15%
    private final double DRV_S_RATE_GAIN_ION_STATION     = 0.003;

    //turn in place variables
    private final static double PID_TURN_THRESHOLD   = 1.25;

	private final static double PID_TURN_IN_PLACE_KP = 0.008;
    
    private final static double TURN_DRIVE_MAX_POS_POWER  =  0.4;
	private final static double TURN_DRIVE_MAX_NEG_POWER  = -0.4;

    private final static double TURN_DRIVE_MIN_POWER = 0.1;

    private final static double TURN_IN_PLACE_PERIOD = 0.010;


	private static double turnCurrentError; 

	private static double turnCurrentAngle;
	private static double turnTargetAngle;

    private Timer autonTimer;
    public CatzLog data;



    public CatzAutonomous()
    {
        autonTimer = new Timer();
    }



    /*-----------------------------------------------------------------------------------------
    *
    *  DriveStraight
    *
    *----------------------------------------------------------------------------------------*/

    public void DriveStraight(double distanceInch, double directionDeg, double maxTime)
    {
        double distanceRemainInch    = 0.0;
        double distanceRemainAbsInch = 0.0;

        double drvPowerDirection = 0.0;
        double drvPower      = 0.0;
        double drvPowerKp    = 0.0;
        double drvPowerClamp = 0.0;

        double turnPower    = 0.0;
        double angleKpPower = 0.0;
        double angleKdPower = 0.0;

        double angleError     = 0.0;
        double prevAngleError = 0.0;
        double angleErrorRate = 0.0;

        double time     = 0.0;
        double prevTime = -1.0;

        double deltaPositionCnts  = 0.0;

        double currentAngle     = 0.0;
        double startingAngle    = 0.0;

        boolean done = false;

        //Robot.drivetrain.LT_FRNT_MODULE.resetDrvDistance();
        deltaPositionCnts = 0.0;

        startingAngle         = 
        distanceRemainInch    = distanceInch;
        distanceRemainAbsInch = Math.abs(distanceRemainInch);

        autonTimer.reset();
        autonTimer.start();

        while(!done)
        {
            time = autonTimer.get();

            if(time > maxTime)
            {
                done = true;
                //Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle()); TBD
            }
            else
            {
                currentAngle = drivetrain.getGyroAngle();
                
                angleError = startingAngle - currentAngle;

                if(prevTime == -1.0)
                {
                    angleErrorRate = 0.0;
                }
                else
                {
                    angleErrorRate = (angleError - prevAngleError) / (time - prevTime);
                }
    
                //deltaPositionCnts   = Robot.drivetrain.LT_FRNT_MODULE.getDrvDistance();
                distanceRemainInch    = distanceInch - (deltaPositionCnts * DRVTRAIN_ENC_COUNTS_TO_INCH);
                distanceRemainAbsInch = Math.abs(distanceRemainInch);
                drvPowerDirection     = Math.signum(distanceRemainInch);

                if(distanceRemainAbsInch <= DRV_S_THRESHOLD_INCH)
                {
                    done = true;
                    //Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle()); TBD
                }
                else
                {
                    drvPowerKp    = (DRV_S_KP * distanceRemainAbsInch);
                    drvPowerClamp = Clamp(DRV_S_MIN_POWER, drvPowerKp, DRV_S_MAX_POWER);
                    drvPower      = drvPowerClamp * Math.signum(drvPowerDirection);


                    angleKpPower = DRV_S_ERROR_GAIN * angleError;
                    angleKdPower = DRV_S_RATE_GAIN  * angleErrorRate;

                    turnPower = Clamp(-1.0, angleKpPower + angleKdPower, 1.0);
                    turnPower *= Math.signum(drvPowerDirection);
                    

                    //Robot.drivetrain.translateTurn(directionDeg, drvPower, turnPower, Robot.drivetrain.getGyroAngle());
                    prevTime       = time;
                    prevAngleError = angleError;

                }
            }
            if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_DRV_STRAIGHT)
            {
               data = new CatzLog(time, deltaPositionCnts, distanceRemainInch,
                                                            drvPowerKp,
                                                            drvPowerClamp, 
                                                            drvPower, 
                                                            currentAngle,
                                                            angleError, 
                                                            angleErrorRate, 
                                                            angleKpPower,
                                                            angleKdPower,
                                                            turnPower,
                                                            -999.0,
                                                            //Robot.drivetrain.LT_FRNT_MODULE.getAngle(),
                                                            0.0, 0.0, 0);  
                Robot.dataCollection.logData.add(data);
            }
         
            Timer.delay(DRV_S_THREAD_PERIOD);
        }

       // Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
    }

    public void DriveStraightOFFChargeStation(double distanceInch, double directionDeg, double maxTime)
    {
        double distanceRemainInch    = 0.0;
        double distanceRemainAbsInch = 0.0;

        double drvPowerDirection = 0.0;
        double drvPower      = 0.0;
        double drvPowerKp    = 0.0;
        double drvPowerClamp = 0.0;

        double turnPower    = 0.0;
        double angleKpPower = 0.0;
        double angleKdPower = 0.0;

        double angleError     = 0.0;
        double prevAngleError = 0.0;
        double angleErrorRate = 0.0;

        double time     = 0.0;
        double prevTime = -1.0;

        double deltaPositionCnts  = 0.0;

        double currentAngle     = 0.0;
        double startingAngle    = 0.0;

        boolean done = false;

        //Robot.drivetrain.LT_FRNT_MODULE.resetDrvDistance();
        deltaPositionCnts = 0.0;

        startingAngle         = drivetrain.getGyroAngle();
        distanceRemainInch    = distanceInch;
        distanceRemainAbsInch = Math.abs(distanceRemainInch);

        autonTimer.reset();
        autonTimer.start();

        while(!done)
        {
            time = autonTimer.get();

            if(time > maxTime)
            {
                done = true;
                //Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
            }
            else
            {
                currentAngle = drivetrain.getGyroAngle();
                
                angleError = startingAngle - currentAngle;

                if(prevTime == -1.0)
                {
                    angleErrorRate = 0.0;
                }
                else
                {
                    angleErrorRate = (angleError - prevAngleError) / (time - prevTime);
                }
    
                //deltaPositionCnts   = Robot.drivetrain.LT_FRNT_MODULE.getDrvDistance();
                distanceRemainInch    = distanceInch - (deltaPositionCnts * DRVTRAIN_ENC_COUNTS_TO_INCH);
                distanceRemainAbsInch = Math.abs(distanceRemainInch);
                drvPowerDirection     = Math.signum(distanceRemainInch);

                if(distanceRemainAbsInch <= DRV_S_THRESHOLD_INCH)
                {
                    done = true;
                    //Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
                }
                else
                {
                    drvPowerKp    = (DRV_S_KP * distanceRemainAbsInch);
                    drvPowerClamp = Clamp(DRV_S_MIN_POWER, drvPowerKp, DRV_S_MAX_POWER_CHARGE_STATION);
                    drvPower      = drvPowerClamp * Math.signum(drvPowerDirection);


                    angleKpPower = DRV_S_ERROR_GAIN * angleError;
                    angleKdPower = DRV_S_RATE_GAIN  * angleErrorRate;

                    turnPower = Clamp(-1.0, angleKpPower + angleKdPower, 1.0);
                    turnPower *= Math.signum(drvPowerDirection);
                    

                    //Robot.drivetrain.translateTurn(directionDeg, drvPower, turnPower, Robot.drivetrain.getGyroAngle());
                    prevTime       = time;
                    prevAngleError = angleError;

                }
            }
            if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_DRV_STRAIGHT)
            {
               data = new CatzLog(time, deltaPositionCnts, distanceRemainInch,
                                                            drvPowerKp,
                                                            drvPowerClamp, 
                                                            drvPower, 
                                                            currentAngle,
                                                            angleError, 
                                                            angleErrorRate, 
                                                            angleKpPower,
                                                            angleKdPower,
                                                            turnPower,
                                                            -999.0,
                                                            //Robot.drivetrain.LT_FRNT_MODULE.getAngle(),
                                                            0.0, 0.0, 0);  
                Robot.dataCollection.logData.add(data);
            }
         
            Timer.delay(DRV_S_THREAD_PERIOD);
        }

        //Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
    }

    /*-----------------------------------------------------------------------------------------
    *
    *  Drivestright on to charge station(modified pid values)
    *
    *----------------------------------------------------------------------------------------*/
    public void DriveStraightONChargeStationFromBack(double distanceInch, double directionDeg, double maxTime)
    {
        double distanceRemainInch    = 0.0;
        double distanceRemainAbsInch = 0.0;

        double drvPowerDirection = 0.0;
        double drvPower      = 0.0;
        double drvPowerKp    = 0.0;
        double drvPowerClamp = 0.0;

        double turnPower    = 0.0;
        double angleKpPower = 0.0;
        double angleKdPower = 0.0;

        double angleError     = 0.0;
        double prevAngleError = 0.0;
        double angleErrorRate = 0.0;

        double time     = 0.0;
        double prevTime = -1.0;

        double deltaPositionCnts  = 0.0;

        double currentAngle     = 0.0;
        double startingAngle    = 0.0;

        boolean done = false;

        //Robot.drivetrain.LT_FRNT_MODULE.resetDrvDistance();
        deltaPositionCnts = 0.0;

        startingAngle         = drivetrain.getGyroAngle();
        distanceRemainInch    = distanceInch;
        distanceRemainAbsInch = Math.abs(distanceRemainInch);

        autonTimer.reset();
        autonTimer.start();

        while(!done)
        {
            time = autonTimer.get();

            if(time > maxTime)
            {
                done = true;
                //Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
            }
            else
            {
                currentAngle = drivetrain.getGyroAngle();
                
                angleError = startingAngle - currentAngle;

                if(prevTime == -1.0)
                {
                    angleErrorRate = 0.0;
                }
                else
                {
                    angleErrorRate = (angleError - prevAngleError) / (time - prevTime);
                }
    
               // deltaPositionCnts   = Robot.drivetrain.LT_FRNT_MODULE.getDrvDistance();
                distanceRemainInch    = distanceInch - (deltaPositionCnts * DRVTRAIN_ENC_COUNTS_TO_INCH);
                distanceRemainAbsInch = Math.abs(distanceRemainInch);
                drvPowerDirection     = Math.signum(distanceRemainInch);

                if(distanceRemainAbsInch <= DRV_S_THRESHOLD_INCH)
                {
                    done = true;
                    //Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
                }
                else
                {
                    drvPowerKp    = (DRV_BACK_KP * distanceRemainAbsInch);
                    drvPowerClamp = Clamp(DRV_S_MIN_POWER_ROLL_BACK, drvPowerKp, DRV_S_MAX_POWER_ROLL_BACK);
                    drvPower      = drvPowerClamp * Math.signum(drvPowerDirection);


                    angleKpPower = DRV_S_ERROR_GAIN * angleError;
                    angleKdPower = DRV_S_RATE_GAIN  * angleErrorRate;

                    turnPower = Clamp(-1.0, angleKpPower + angleKdPower, 1.0);
                    turnPower *= Math.signum(drvPowerDirection);
                    

                    //Robot.drivetrain.translateTurn(directionDeg, drvPower, turnPower, Robot.drivetrain.getGyroAngle());
                    prevTime       = time;
                    prevAngleError = angleError;

                }
            }
            if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_BALANCE)
            {
               data = new CatzLog(time, deltaPositionCnts, distanceRemainInch,
                                                            drvPowerKp,
                                                            drvPowerClamp, 
                                                            drvPower, 
                                                            currentAngle,
                                                            angleError, 
                                                            angleErrorRate, 
                                                            angleKpPower,
                                                            angleKdPower,
                                                            turnPower,
                                                            -999.0,
                                                            //Robot.drivetrain.LT_FRNT_MODULE.getAngle(),
                                                            //Robot.navX.getRoll()
                                                            -999.0, 0.0, 0);  
                Robot.dataCollection.logData.add(data);
            }
         
            Timer.delay(DRV_S_THREAD_PERIOD);
        }

        //Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
    }

    /*-----------------------------------------------------------------------------------------
    *
    *  TurnInPlace
    *
    *----------------------------------------------------------------------------------------*/
    public void TurnInPlace(double degreesToTurn, double timeoutSeconds)
    {
        boolean turnInPlaceDone = false;

        double  currentTime       = 0.0;
        double  angleRemainingAbs = 999.0;
        double  turnPower = 0.0;

        autonTimer.reset();
        autonTimer.start(); 

        turnCurrentAngle  = drivetrain.getGyroAngle();
        turnTargetAngle   = degreesToTurn + turnCurrentAngle;

        while (turnInPlaceDone == false)
        {
            currentTime  = autonTimer.get();
            turnCurrentAngle = drivetrain.getGyroAngle();
    
            // calculate error
            turnCurrentError      = turnTargetAngle - turnCurrentAngle;
            angleRemainingAbs = Math.abs(turnCurrentError);

            if (angleRemainingAbs < PID_TURN_THRESHOLD) 
            { 
                turnInPlaceDone = true;
                //Robot.drivetrain.rotateInPlace(0.0);
            }
            else
            {
                if (currentTime > timeoutSeconds) 
                {
                    turnInPlaceDone = true;
                    //Robot.drivetrain.rotateInPlace(0.0);
                } 
                else
                {
                    turnPower = turnCurrentError * PID_TURN_IN_PLACE_KP;

                    //Clamp
                    //MAX POWER
                    if(turnPower >= TURN_DRIVE_MAX_POS_POWER)
                    {
                        turnPower = TURN_DRIVE_MAX_POS_POWER;
                    } 
                    else if(turnPower <= TURN_DRIVE_MAX_NEG_POWER)
                    {
                        turnPower = TURN_DRIVE_MAX_NEG_POWER;
                    }

                    //MIN POWER
                  if (Math.abs(turnPower) < TURN_DRIVE_MIN_POWER)
                    {
                        turnPower = Math.signum(turnPower) * TURN_DRIVE_MIN_POWER;
                    }
                    
                    //Robot.drivetrain.rotateInPlace(turnPower);
                }
            }

            if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_TURN_IN_PLACE)
            {
            data = new CatzLog(currentTime, turnCurrentAngle, turnCurrentError, turnPower, 
                                -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999);

            Robot.dataCollection.logData.add(data);
            }

            Timer.delay(TURN_IN_PLACE_PERIOD);
        }
    }

    public double Clamp(double min, double in, double max)
    {
        if(in > max)
        {
            return max;
        }
        else if(in < min)
        {
            return min;
        }
        else
        {
            return in;
        }
    }
}