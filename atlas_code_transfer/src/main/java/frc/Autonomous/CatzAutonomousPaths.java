package frc.Autonomous;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.arm.CatzArm;
import frc.Mechanisms.drivetrain.CatzDrivetrain;
import frc.Mechanisms.elevator.CatzElevator;
import frc.Mechanisms.intake.CatzIntake;
import frc.robot.CatzConstants;
import frc.robot.Robot;

/*****************************************************************************************
*
* Autonomous selections
* 
*****************************************************************************************/
@SuppressWarnings("unused")
public class CatzAutonomousPaths
{  
    private static CatzDrivetrain drivetrain    = CatzDrivetrain.getInstance();
    private static CatzElevator   elevator      = CatzElevator.getIntstance();
    private static CatzIntake     intake        = CatzIntake.getInstance();
    private static CatzArm        arm           = CatzArm.getInstance();

    public final SendableChooser<Enum> chosenAllianceColor = new SendableChooser<>();
    private final SendableChooser<Integer> chosenPath         = new SendableChooser<>();

    /*------------------------------------------------------------------------------------
    *  Field Relative angles when robot is TBD - Finish Comment  
    *-----------------------------------------------------------------------------------*/
    private final double BIAS_OFFSET =   0.0;
    private final double FWD_OR_BWD  =   0.0 + BIAS_OFFSET;
    private final double RIGHT       =   90.0; 
    private final double LEFT        =   -90.0;

    /*------------------------------------------------------------------------------------
    *  Path ID's
    *-----------------------------------------------------------------------------------*/
    private final int LEFT_SCORE_1_LOW_CUBE               = 1;
    private final int LEFT_SCORE_2_LOW_CUBE               = 2;
    private final int LEFT_SCORE_1_BALANCE_LOW_CUBE       = 3;

    private final int LEFT_SCORE_1_HIGH_CONE              = 4;
    private final int LEFT_SCORE_2_HIGH_CONE              = 5;
    private final int LEFT_SCORE_1_BALANCE_HIGH_CONE      = 6;

    private final int CENTER_SCORE_1_LOW_BALANCE          = 20;
    private final int CENTER_SCORE_1_MID_BALANCE          = 21;
    private final int CENTER_SCORE_1_HIGH_BALANCE         = 22;

    private final int RIGHT_SCORE_1_LOW_CUBE                       = 40;
    private final int RIGHT_SCORE_2_LOW_CUBE                       = 41;
    private final int RIGHT_SCORE_1_BALANCE_LOW_CUBE               = 42;

    private final int RIGHT_SCORE_1_HIGH_CONE             = 43;
    private final int RIGHT_SCORE_2_HIGH_CONE             = 44;
    private final int RIGHT_SCORE_1_BALANCE_HIGH_CONE     = 45;

    private final int TEST                            = 100;



    public static int pathID;

    public static int autoState = Robot.COMMAND_STATE_NULL;

    public static Thread autoThread;

     /*  DRIVE STRAIGHT VALUES: 
     * if distance > 70, then FAST, else SLOW
     * 8 second maxTime is an arbitrary number, subject to change upon more testing 
     * only robot backwards movement has negative signs over distance and maxspeed
     * left and right distances and max speed aren't negative
     * TEMP_DECEL_DIST decelDistance is an arbitrary number, subject to change upon more testing
     * 
     *   *note* - autonomous is 15 seconds, meaning that all of this will have to finsih within that time
     *          - THIS CODE IS MADE FOR BLUE SIDE 
     *          - FOR RED, CHANGE LEFTS WITH RIGHTS AND RIGHTS WITH LEFTS (from blue)
     *          - movement similar to code.org level programming
    */

    /* PATH NAME:
     *    /CenterRightTunnel/
     * - CenterRight (Starting Position)
     * - Tunnel (type of movement/movement path)
     */

    /* Distances:          -______-
     * drive.DriveStraight(distance, decelDistance, maxSpeed, wheelPos, maxTime);
     *  - 224 = distance from grid to center pieces
     *                
     */
    // drive.DriveStraight(distance, decelDist, )


    public CatzAutonomousPaths()
    {
        chosenAllianceColor.setDefaultOption("Blue Alliance", CatzConstants.AllianceColor.BlUE_ALLIANCE);
        chosenAllianceColor.addOption       ("Red Alliance",  CatzConstants.AllianceColor.RED_ALLIANCE);
        SmartDashboard.putData              ("Alliance Color", chosenAllianceColor);

        chosenPath.setDefaultOption("Left Score 1 low cube",           LEFT_SCORE_1_LOW_CUBE);   
        chosenPath.addOption       ("Left Score 2 low cube",           LEFT_SCORE_2_LOW_CUBE);
        chosenPath.addOption       ("Left Score 1 Balance low cube",   LEFT_SCORE_1_BALANCE_LOW_CUBE);

        chosenPath.addOption       ("Left Score 1 high Cone",           LEFT_SCORE_1_HIGH_CONE);
        chosenPath.addOption       ("Left Score 2 high cone",           LEFT_SCORE_2_HIGH_CONE);
        chosenPath.addOption       ("Left Score 1 Balance high cone",   LEFT_SCORE_1_BALANCE_HIGH_CONE);

        chosenPath.addOption       ("Center Score 1 Low",          CENTER_SCORE_1_LOW_BALANCE);
        chosenPath.addOption       ("Center Score 1 Mid Balance",  CENTER_SCORE_1_MID_BALANCE);
        chosenPath.addOption       ("Center Score 1 High Balance", CENTER_SCORE_1_HIGH_BALANCE);

        chosenPath.addOption       ("Right Score 1 low cube",          RIGHT_SCORE_1_LOW_CUBE);  //TBD more paths created during comp,should we revert back? How does trajectory play into auton?
        chosenPath.addOption       ("Right Score 2 low cube",          RIGHT_SCORE_2_LOW_CUBE);
        chosenPath.addOption       ("Right Score 1 Balance low cube",  RIGHT_SCORE_1_BALANCE_LOW_CUBE);

        chosenPath.addOption       ("Right Score 1 high cone",          RIGHT_SCORE_1_HIGH_CONE);
        chosenPath.addOption       ("Right Score 2 high cone",          RIGHT_SCORE_2_HIGH_CONE);
        chosenPath.addOption       ("Right Score 1 Balance high cone",  RIGHT_SCORE_1_BALANCE_HIGH_CONE);


        chosenPath.addOption       ("TEST PATH",  TEST);

        SmartDashboard.putData     ("Auton Path", chosenPath);
    
    }


    public void executeSelectedPath()
    {
        pathID = chosenPath.getSelected();

        System.out.println("PathID: " + pathID);

        switch (pathID)
        {
            case LEFT_SCORE_1_LOW_CUBE: sideScore1LowCube(); //Scores High Cone - TBD
            break;

            case LEFT_SCORE_2_LOW_CUBE: LeftScore2LowCube(); //Scores High Cone + Low Cone - TBD
            break;

            case LEFT_SCORE_1_HIGH_CONE: sideScore1HighCone();
            break;

            case LEFT_SCORE_2_HIGH_CONE: LeftScore2HighCone();
            break;


            //case LEFT_SCORE_1_BALANCE: LeftScore1Balance(); //Scores High Cone - TBD
            //break;
            

            case CENTER_SCORE_1_LOW_BALANCE: centerScore1LowCubeBalance(); //Scores Low Cube - TBD
            break;

            case CENTER_SCORE_1_MID_BALANCE: centerScore1MidConeBalance();  //Scores Mid Cone - TBD
            break;

            case CENTER_SCORE_1_HIGH_BALANCE: centerScore1HighConeBalance(); //Scores High Cone - TBD
            break;


            case RIGHT_SCORE_1_LOW_CUBE: sideScore1LowCube(); //Scores High Cone - TBD
            break;

            case RIGHT_SCORE_2_LOW_CUBE: RightScore2LowCube(); //Scores High Cone + Low Cube - TBD
            break;

            case RIGHT_SCORE_1_HIGH_CONE: sideScore1HighCone();
            break;

            case RIGHT_SCORE_2_HIGH_CONE: RightScore2HighCone();
            break;


          //  case RIGHT_SCORE_1_BALANCE: RightScore1Balance(); //Scores High Cone - TBD
          //  break;

            case TEST: testPath(); //Scores High Cone - TBD
            break;
        }

    }


    public void testPath()
    {
        Balance();
    }



    /*-----------------------------------------------------------------------------------------
    *    
    *  Auton Functions
    * 
    *----------------------------------------------------------------------------------------*/
    public void Balance()
    {
        Robot.balance.StartBalancing();
    }

    /*-----------------------------------------------------------------------------------------
    *    
    *  Auton Paths
    * 
    *----------------------------------------------------------------------------------------*/
    public void sideScore1LowCube()
    {
        scoreCubeLow();

        Robot.auton.DriveStraight(200, FWD_OR_BWD,  4.0);     //From Grid to exit community
    }

    public void sideScore1HighCone()
    {
        scoreConeHigh();

        Robot.auton.DriveStraight(200, FWD_OR_BWD,  4.0);     //From Grid to exit community
    }

    public void SidePickup(){
        double direction;

        if(chosenAllianceColor.getSelected() == CatzConstants.AllianceColor.RED_ALLIANCE)
        {
            direction = RIGHT;
        }
        else
        {
            direction = LEFT;
        }

        Robot.auton.DriveStraight( 30, FWD_OR_BWD, 2.0);     //From Grid to area to do 180 deg turn

        Robot.auton.TurnInPlace(180, 2.0);

        pickUpCube();
        
        Robot.auton.DriveStraight(176, FWD_OR_BWD, 5.0);  
        Timer.delay(0.1); 
        // stow();

        if(direction == 90.0)
        {
            Robot.auton.TurnInPlace(-180, 2.0);
        }
        else
        {
            Robot.auton.TurnInPlace(180, 2.0);
        }
        stow();

        Robot.auton.DriveStraight(-212, FWD_OR_BWD, 5.0);
        Robot.auton.DriveStraight( 36,  direction, 2.0);
    }
    
    public void LeftScore2LowCube()
    {
        scoreCubeLow();
        SidePickup();
        scoreCubeLow();
    }

    public void LeftScore2HighCone()
    {
        scoreConeHigh();
        SidePickup();
        scoreCubeHigh();
    }

    public void RightScore2LowCube()
    {
        scoreCubeLow();
        SidePickup();
        scoreCubeLow();
    }
        
    public void RightScore2HighCone()
    {
        scoreCubeHigh();
        SidePickup();
        scoreCubeHigh();
    }
/* 
    public void LeftScore1Balance()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = RIGHT;
        }
        else
        {
            direction = LEFT;
        }

        sideScore1Pickup1(direction);

        Robot.auton.DriveStraight(-42, FWD_OR_BWD, 5.0);
        Robot.auton.DriveStraight(60, direction, 2.0);
        Robot.auton.DriveStraight(-80, FWD_OR_BWD, 2.0);
        Balance();
    }


    public void RightScore1Balance()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = LEFT;
        }
        else
        {
            direction = RIGHT;
        }

        sideScore1Pickup1(direction);

        Robot.auton.DriveStraight(-42, FWD_OR_BWD, 5.0);
        Robot.auton.DriveStraight(48, direction, 2.0);
        Robot.auton.DriveStraight(-80, FWD_OR_BWD, 2.0);
        Balance();

    }
    */

    public void centerMobilityBalance() 
    {
        Robot.auton.DriveStraightOFFChargeStation(170, FWD_OR_BWD, 4.0);
        Timer.delay(1.0);//wait for balance to level out
        Robot.auton.DriveStraightONChargeStationFromBack(-108, FWD_OR_BWD, 4.0); 

        Balance();
    }

    public void centerScore1MidConeBalance() 
    {
        scoreConeMid();
        centerMobilityBalance();
    }
    
    public void centerScore1HighConeBalance() 
    {
        scoreConeHigh();
        centerMobilityBalance();
    }
    
    public void centerScore1LowCubeBalance() 
    {
        scoreCubeLow();
        //ejectCube();
        centerMobilityBalance();
    }




    public void scoreConeLow()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_LOW_CONE, Robot.GP_CONE);

        scoreCone();
    }

    public void scoreConeMid()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_MID_CONE, Robot.GP_CONE);
        Timer.delay(0.5);
        scoreCone();
    }

    public void scoreConeHigh()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_HIGH_CONE, Robot.GP_CONE);
        scoreCone();
    }


    public void scoreCubeLow()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_LOW_CUBE, Robot.GP_CUBE);

        scoreCube();
    }

    public void scoreCubeMid()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_MID_CUBE, Robot.GP_CUBE);

        scoreCube();
    }

    public void scoreCubeHigh()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE, Robot.GP_CUBE);

        scoreCube();
    }

    public void scoreCube()
    {
        ejectCube();
        setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
        intake.rollersOff();
    }

    public void scoreCone()
    {
        ejectCone();
        setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
        intake.rollersOff();
    }

    public void ejectCube()
    {
        intake.rollersOutCube();
        Timer.delay(0.1); //TBD will need to change
    }

    public void ejectCone()
    {
        intake.rollersOutCone();
        Timer.delay(0.2);
        
    }



    public void setCommandStateAuton(int cmdState, int gamePiece)
    {
        
        int timeout = 0;
        boolean done = false;

        boolean elevatorInPos;
        boolean armInPos;
        boolean intakeInPos;

        Robot.selectedGamePiece = gamePiece;

        if(cmdState != Robot.COMMAND_UPDATE_STOW)
        {
            while(!done)
            {
                CatzArm.getInstance().armPeriodic();
                CatzElevator.getIntstance().elevatorPeriodic();
                CatzIntake.getInstance().intakePeriodic();
                
                elevator.cmdProcElevator(0.0,   false, cmdState);
                arm.cmdProcArm          (false, false, cmdState);
        
                if(cmdState == Robot.COMMAND_UPDATE_STOW)
                {
                    intake.cmdProcIntake(0.0, false, true, false, false, cmdState, gamePiece);
                }
                else
                {
                    intake.cmdProcIntake(0.0, false, false, false, false, cmdState, gamePiece);
                }

                elevatorInPos = elevator.isElevatorInPos();
                armInPos      = arm.isArmInPos();
                intakeInPos   = intake.isIntakeInPos();

                if(elevatorInPos && armInPos && intakeInPos)
                {
                    done = true;
                }

                timeout++;
                if(timeout >= 150)
                {
                    done = true;
                    System.out.println("Timed Out! \n" + 
                                       "Elev in position: " + elevatorInPos +
                                       " Arm in position: " + armInPos +
                                       " Intk in position: " + intakeInPos +
                                       " Elev enc pos: " + elevator.getElevatorEncoder() +
                                       " Arm enc pos: " + arm.getArmEncoder() +
                                       " Intk enc pos: " + intake.getWristPosition()
                                       );
                }

                Timer.delay(0.020);
            }
        }
    }

    public void pickUpCone()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE, Robot.GP_CONE);
        intake.rollersInCone();
    }

    public void pickUpCube()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE, Robot.GP_CUBE);
        intake.rollersInCube();
    }

    public void stow()
    {
        intake.rollersOff();
        setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
    }
}