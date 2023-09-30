// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.CatzRGB;
import frc.Mechanisms.ColorMethod;
import frc.Mechanisms.AbstractMechanism;
import frc.Mechanisms.Odometry.CatzRobotTracker;
import frc.Mechanisms.arm.CatzArm;
import frc.Mechanisms.drivetrain.CatzDrivetrain;
import frc.Mechanisms.elevator.CatzElevator;
import frc.Mechanisms.intake.CatzIntake;
import frc.Autonomous.*;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot 
{

  private static final CatzDrivetrain drivetrain = CatzDrivetrain.getInstance();
  private static final CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private static final CatzElevator elevator = CatzElevator.getIntstance();
  private static final CatzIntake intake = CatzIntake.getInstance();
  private static final CatzArm    arm = CatzArm.getInstance();


  //---------------------------------------------------------------------------------------------
  //  Shared Libraries & Utilities
  //---------------------------------------------------------------------------------------------

  public static DataCollection      dataCollection;
  public ArrayList<CatzLog>         dataArrayList;

  //----------------------------------------------------------------------------------------------
  //  Shared Robot Components (e.g. not mechanism specific, such as PDH, NavX, etc)
  //----------------------------------------------------------------------------------------------
  public static PowerDistribution PDH;

  public static Timer             currentTime;

  private XboxController xboxDrv;
  private XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  public static final int DPAD_UP = 0;
  public static final int DPAD_DN = 180;
  public static final int DPAD_LT = 270;
  public static final int DPAD_RT = 90;

  public double  xboxElevatorManualPwr    = 0.1;
  public boolean xboxElevatorManualMode = false;
  public boolean xboxStowPos   = false;
  public boolean xboxLowNode   = false;
  public boolean xboxMidNode   = false;
  public boolean xboxHighNode  = false;
  public boolean xboxPickUpGroundPos = false;
  public boolean xboxPickUpSinglePos = false;
  public boolean xboxPickUpDoublePos = false;
  public boolean autoAlignWithCubeNode = false;
  public boolean autoAlignWithConeNode = false;

  public static boolean xboxNULL      = false;


  public static final int COMMAND_STATE_NULL            =  0;

  public static final int COMMAND_UPDATE_PICKUP_GROUND_CONE     = 1;
  public static final int COMMAND_UPDATE_PICKUP_GROUND_CUBE     = 2;
  public static final int COMMAND_UPDATE_PICKUP_SINGLE_CONE     = 3;
  public static final int COMMAND_UPDATE_PICKUP_SINGLE_CUBE     = 4;
  public static final int COMMAND_UPDATE_PICKUP_DOUBLE_CONE     = 5;
  public static final int COMMAND_UPDATE_PICKUP_DOUBLE_CUBE     = 6;

  public static final int COMMAND_UPDATE_STOW            = 7;

  public static final int COMMAND_UPDATE_SCORE_LOW_CONE  = 10;
  public static final int COMMAND_UPDATE_SCORE_LOW_CUBE  = 11;
  public static final int COMMAND_UPDATE_SCORE_MID_CONE  = 12;
  public static final int COMMAND_UPDATE_SCORE_MID_CUBE  = 13;
  public static final int COMMAND_UPDATE_SCORE_HIGH_CONE = 14;
  public static final int COMMAND_UPDATE_SCORE_HIGH_CUBE = 15;

  public static int commandedStateUpdate = COMMAND_STATE_NULL;

  public final static int GP_NULL = 0;
  public final static int GP_CUBE = 1;
  public final static int GP_CONE = 2;

  public static int selectedGamePiece = GP_CUBE;

  public static final int MODE_AUTO        = 0;
  public static final int MODE_MANUAL_HOLD = 1;
  public static final int MODE_MANUAL      = 2;
    

  
  //---------------------------------------------------------------------------------------------
  //  Autonomous
  //---------------------------------------------------------------------------------------------
  public static CatzAutonomous      auton;
  public static CatzAutonomousPaths paths = new CatzAutonomousPaths();
  public static CatzBalance         balance;

  private final double OFFSET_DELAY = 0.5;    //TBD put into AUTO BALANCE class

  //---------------------------------------------------------------------------------------------
  //  Mechanisms
  //---------------------------------------------------------------------------------------------
  public static CatzRGB        led = new CatzRGB();

  //---------------------------------------------------------------------------------------------
  public enum mechMode
  {
    AutoMode(Color.kGreen),
    ManualHoldMode(Color.kCyan),
    ManualMode(Color.kRed);

    public Color color;
    mechMode(Color color){
      this.color = color;
    }
  }

  public enum gamePiece{
    Cube(Color.kPurple),
    Cone(Color.kYellow),
    None(Color.kGhostWhite);

    public Color color;
    gamePiece(Color color){
      this.color = color;
    }
  }

  public enum gameModeLED{
    Autobalancing(led.oneColorFill, Color.kGreen),
    InAutonomous(led.startFlowing, led.PHANTOM_SAPPHIRE, Color.kWhite),
    MatchEnd(led.startFlowingRainbow),
    EndgameWheelLock(led.oneColorFillAllianceColor), 
    TeleOp(led.doNothing);

    public ColorMethod method;
    public Color[] color;
    private gameModeLED(ColorMethod method, Color... color)
    {
      this.method = method;
      this.color = color;
    }
  }

  public static mechMode intakeControlMode = mechMode.AutoMode;
  public static mechMode elevatorControlMode = mechMode.AutoMode;
  public static mechMode armControlMode = mechMode.AutoMode;
  public static gameModeLED currentGameModeLED = gameModeLED.MatchEnd;
  public static gamePiece currentGamePiece = gamePiece.None;


  /*-----------------------------------------------------------------------------------------
  *  
  *  robotXxx
  *
  *----------------------------------------------------------------------------------------*/

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    Logger logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) 
    {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (CatzConstants.currentMode) 
    {
      // Running on a real robot, log to a USB stick
      case REAL:
        logger.addDataReceiver(new WPILOGWriter("/media/sda1/Robotdata"));
        logger.addDataReceiver(new NT4Publisher());
       // new PowerDistribution(1, ModuleType.kRev);
        break;

      // Running a physics simulator, log to local folder
      case SIM:
        logger.addDataReceiver(new WPILOGWriter("F:/robotics code projects/loggingfiles/"));
        logger.addDataReceiver(new NT4Publisher());
        break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    // Start AdvantageKit logger
    logger.start();


    //-----------------------------------------------------------------------------------------
    //  Shared Libraries & Utilities
    //-----------------------------------------------------------------------------------------
    dataCollection = new DataCollection();
    dataArrayList  = new ArrayList<CatzLog>();
    
    dataCollection.dataCollectionInit(dataArrayList);


    //-----------------------------------------------------------------------------------------
    //  Shared Robot Components (e.g. not mechanism specific)
    //-----------------------------------------------------------------------------------------
    PDH = new PowerDistribution();

    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    currentTime = new Timer();

    //----------------------------------------------------------------------------------------------
    //  Autonomous
    //----------------------------------------------------------------------------------------------
    auton   = new CatzAutonomous();
    balance = new CatzBalance();

    System.out.println("Deploy robot code"); 
  }
  @Override
  public void robotPeriodic() 
  {
    if(!DriverStation.isTeleopEnabled() && !DriverStation.isAutonomousEnabled())
    {
      drivetrain.drivetrainPeriodic();
      elevator.elevatorPeriodic();
      arm.armPeriodic();
      intake.intakePeriodic();
    }

    if(DriverStation.isAutonomousEnabled())
    {
      balance.AutoBalanceLoop();
    }


    //----------------------------------------------------------------------------------------------
    //  Update status, LED's
    //----------------------------------------------------------------------------------------------
    dataCollection.updateLogDataID(); 
    led.LEDPeriodic();

    //----------------------------------------------------------------------------------------------
    //  Shuffleboard Data Display
    //----------------------------------------------------------------------------------------------
    SmartDashboard.putNumber("gamepiece int", selectedGamePiece);
    SmartDashboard.putNumber("COMMAND STATE", commandedStateUpdate);


    drivetrain.smartDashboardDriveTrain();
    elevator.smartDashboardElevator();
    elevator.smartDashboardElevator_DEBUG();
    //balance.SmartDashboardBalanceDebug();

        
    arm.smartDashboardARM();
    balance.SmartDashboardBalance();
  
    //debug should be commented out for comp

    //intake.smartdashboardIntakeDebug();      
  }



  /*-----------------------------------------------------------------------------------------
  *  
  *  autonomousXxx
  *
  *----------------------------------------------------------------------------------------*/
  @Override
  public void autonomousInit() 
  {
    
    //drivetrain.setBrakeMode();
    currentTime.reset();
    currentTime.start();

    currentGameModeLED = gameModeLED.InAutonomous;
    
    Timer.delay(OFFSET_DELAY);  //TBD - This should be 

    paths.executeSelectedPath(); 
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
  {

  }



  /*-----------------------------------------------------------------------------------------
  *  
  *  teleopXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit()
  {

    intake.resetPID();
    intake.enablePID(false);
    currentTime.reset();
    currentTime.start();

    dataCollection.startDataCollection();
    //balance.StopBalancing(); could possible affect datacollection

    currentGameModeLED = gameModeLED.TeleOp;
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic()
  {
    //----------------------------------------------------------------------------------------------
    //  Periodic calls that update the inputs(sensors/motor encoders) for each "main" loop iteration 
    //----------------------------------------------------------------------------------------------
    drivetrain.drivetrainPeriodic();
    elevator.elevatorPeriodic();
    arm.armPeriodic();
    intake.intakePeriodic();

    //DriveTrain Proc
    drivetrain.cmdProcSwerve(0.0, xboxDrv.getLeftY()/2, xboxDrv.getRightX(), (xboxDrv.getRightTriggerAxis() > 0.9));

    if(xboxDrv.getStartButtonPressed())
    {
      drivetrain.zeroGyro();
    }


    xboxHighNode           = xboxAux.getYButton();
    xboxMidNode            = xboxAux.getBButton();
    xboxLowNode            = xboxAux.getAButton();
    xboxStowPos            = xboxAux.getXButton()     | xboxDrv.getRightStickButton();
    xboxPickUpGroundPos    = xboxAux.getStartButton() | xboxDrv.getLeftStickButton();

    xboxElevatorManualMode = xboxAux.getRightStickButton();
    xboxElevatorManualPwr  = xboxAux.getRightY();



    //stateMachhine Proc
    xboxGamePieceSelection(xboxAux.getPOV(),                // Left = Cone, Right = Cube
                           xboxAux.getBackButtonPressed()); // Clear Selected Game Piece

    determineCommandState(xboxLowNode, 
                          xboxMidNode, 
                          xboxHighNode, 
                          xboxStowPos,
                          xboxPickUpGroundPos,
                          xboxAux.getPOV() == DPAD_DN,
                          false);
  
                     
    //Mechanism Procs
    elevator.cmdProcElevator(xboxElevatorManualPwr,  // Manual and Manual Hold Elevator Power
                            xboxElevatorManualMode,  // Enter Manual Mode
                            commandedStateUpdate);
                            
    arm.cmdProcArm(xboxAux.getRightTriggerAxis() >= 0.1,   //Manual Extend Arm
                   xboxAux.getLeftTriggerAxis()  >= 0.1,   //Manual Retract Arm 
                   commandedStateUpdate); 

    intake.cmdProcIntake( xboxAux.getLeftY(),                   //Semi-manual override 
                          xboxAux.getRightBumper() | xboxDrv.getRightBumper(),             //Roller in 
                          xboxAux.getLeftBumper() | xboxDrv.getLeftBumper(),              //Roller out
                          xboxAux.getLeftStickButtonPressed(),  //Enter all-manual mode
                          (xboxAux.getRightBumper() & xboxAux.getLeftBumper()),  //Soft limit override
                          commandedStateUpdate,
                          selectedGamePiece);
                         
    commandedStateUpdate = COMMAND_STATE_NULL;



    
    // Lock Wheels (Balancing)
    if(xboxDrv.getBButton())
    {
      drivetrain.lockWheels(); 
    }

    /*
    if(DriverStation.getMatchTime() < 2.0)
    {
      led.matchDone = true;

    }
    else if(DriverStation.getMatchTime() < 15.0) //TBD commented out during comp?
    {
      led.endGame = true;
    }
    */

  }



  /*-----------------------------------------------------------------------------------------
  *  
  *  disabledXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {

    
    System.out.println( "intake temp " + intake.intakeWristTemp());
 
    currentGameModeLED = gameModeLED.MatchEnd;

    currentTime.stop();
    
    if(dataCollection.logDataValues == true)
    {
      dataCollection.stopDataCollection();

      try 
      {
        dataCollection.exportData(dataArrayList);
      } 
      catch (Exception e) 
      {
        e.printStackTrace();
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic()
  {

  }


  /*-----------------------------------------------------------------------------------------
  *  
  *  testXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  /*-----------------------------------------------------------------------------------------
  *  
  *  simulateXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  /*-----------------------------------------------------------------------------------------
  *  
  *  Misc
  *
  *----------------------------------------------------------------------------------------*/  
  public void determineCommandState(boolean LowNode,
                                    boolean MidNode,
                                    boolean HighNode,
                                    boolean StowPos,
                                    boolean PickUpGroundPos,
                                    boolean PickUpSinglePos,
                                    boolean PickUpDoublePos) 
  {
    if(StowPos)
    {
      commandedStateUpdate = COMMAND_UPDATE_STOW;
    }
    else if (LowNode)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_LOW_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_LOW_CONE;
      }
    }
    else if(MidNode)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_MID_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_MID_CONE;
      }
    }
    else if(HighNode)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_HIGH_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_HIGH_CONE;
      }
    }
    else if(PickUpGroundPos)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_GROUND_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_GROUND_CONE;
      }
    }
    else if(PickUpSinglePos)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_SINGLE_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_SINGLE_CONE;
      }

    }
    else if(PickUpDoublePos)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_DOUBLE_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_DOUBLE_CONE;
      }
    }
  }   //end of determineCommandState()


  public void xboxGamePieceSelection(double Dpad, boolean SelectButtonPressed)
  {
    if(Dpad == DPAD_LT)
    {
      selectedGamePiece = GP_CONE;
      currentGamePiece = gamePiece.Cone;
    }
    else if(Dpad == DPAD_RT)
    {
      selectedGamePiece = GP_CUBE;
      currentGamePiece = gamePiece.Cube;
    }
    else if(SelectButtonPressed)
    {
      selectedGamePiece = GP_NULL;
      currentGamePiece = gamePiece.None;
    }
  }
  
    public static double getElevatorEncoderReal()
    {
      return elevator.getElevatorEncoder();
    }

}