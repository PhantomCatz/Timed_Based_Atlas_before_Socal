package frc.Autonomous;

import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.drivetrain.CatzDrivetrain;
import frc.robot.Robot;

public class CatzBalance
{
    private CatzDrivetrain drivetrain = CatzDrivetrain.getInstance();
    
    public static Timer timer = new Timer();
    public static double prevTime = 0.0;
    public static double time = 0.0;

    public  Boolean startBalance = false;

    public static double prevBalanceAngle = 0.0;
    public static double balanceAngle = 0.0;
    public static double angleRate = 0.0;
    public static double power = 0.0;
    public static double angleTerm = 0.0;
    public static double rateTerm = 0.0;
    public static double powerFinal = 0.0;

    public static CatzLog data;

    public final double ANG_SLOWBAND = 10.0; 
    public final double ANG_GAIN = 0.008; //0.007
    public final double RATE_GAIN = 0.002; //0.002 //TBD which values are the most optimal?
    public final double MAX_POWER = 0.30;
    public final double BALANCE_THREAD_PERIOD = 0.02;

    public CatzBalance()
    {

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

    public void AutoBalanceLoop()
    {
            
            if(startBalance)
            {
                time = timer.get();

                balanceAngle = drivetrain.getGyroYaw(); 

                if(prevTime < 0.0)
                {
                    angleRate = 0.0;
                } 
                else 
                {
                    angleRate = (balanceAngle - prevBalanceAngle)/(time - prevTime);
                }

                // PID without the I
                angleTerm = balanceAngle * ANG_GAIN;
                rateTerm = angleRate * RATE_GAIN;

                power = Clamp(-MAX_POWER, angleTerm + rateTerm, MAX_POWER);

                if(Math.abs(power)< 0.07)
                {
                    if(power < 0)
                    {
                        power = -0.04;
                    }
                    else if(power > 0)
                    {
                        power = 0.04;
                    }
                    
                    if(Math.abs(balanceAngle) < 2.0)
                    {
                        power = 0.0;
                    }
                }
            
                
                drivetrain.drive(0.0, -power, 0.0); 
                prevBalanceAngle = balanceAngle;
                prevTime = time;

                Logger.getInstance().recordOutput("Autobalance/power", power);
                Logger.getInstance().recordOutput("Autobalance/balanceAngle", balanceAngle);
                Logger.getInstance().recordOutput("Autobalance/angleRate", angleRate);
                Logger.getInstance().recordOutput("Autobalance/rateTerm", rateTerm);

                if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_BALANCE)
                {
                    data = new CatzLog(time, balanceAngle, angleRate, power, powerFinal, angleTerm, rateTerm, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);  
                    Robot.dataCollection.logData.add(data);
                }
            }

    }

    public void StartBalancing()
    {
        timer.reset();
        timer.start();
        
        prevTime = -1.0;
        prevBalanceAngle = 0.0;

        startBalance = true;

        Robot.currentGameModeLED = Robot.gameModeLED.Autobalancing;
    }

    public void StopBalancing()
    {
        startBalance = false;
    }

    public void SmartDashboardBalanceDebug()
    {
        SmartDashboard.putNumber("Pitch Rate", angleRate);
        SmartDashboard.putNumber("Power", power);
    }

    public void SmartDashboardBalance()
    {
        SmartDashboard.putNumber("Pitch", balanceAngle);
    }
}