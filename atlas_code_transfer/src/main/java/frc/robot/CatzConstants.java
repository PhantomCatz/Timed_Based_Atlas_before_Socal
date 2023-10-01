/***
 * CatzConstants
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is where reusable constants are defined
 * - should all constants move here?
 ***/

package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class CatzConstants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  //--------------------Alliance color---------------------------
  public static enum AllianceColor {
    BlUE_ALLIANCE,
    RED_ALLIANCE
  }
  //------------------------Autonomous path Enums--------------------

  public static enum AutonomousPath 
  {
    TEST

  }
  
  //----------------------Drive Constants---------------------------
  public static final class DriveConstants
  {
    private static final double MODULE_DISTANCE_FROM_CENTER = 0.42672;

    private static final Translation2d SWERVE_LEFT_FRONT_LOCATION  = new Translation2d(MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER).div(Math.sqrt(2));
    private static final Translation2d SWERVE_LEFT_BACK_LOCATION   = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER).div(Math.sqrt(2));
    private static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER).div(Math.sqrt(2));
    private static final Translation2d SWERVE_RIGHT_BACK_LOCATION  = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER).div(Math.sqrt(2));

    // calculates the orientation and speed of individual swerve modules when given the motion of the whole robot
    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        SWERVE_LEFT_FRONT_LOCATION,
        SWERVE_LEFT_BACK_LOCATION,
        SWERVE_RIGHT_FRONT_LOCATION,
        SWERVE_RIGHT_BACK_LOCATION
    );

    public static final double MAX_SPEED = 4.0; // meters per second
    public static final double MAX_ANGSPEED = 3.0; // radians per second

    public static final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio reduction
    public static final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio reduction
    
    public static final double DRVTRAIN_WHEEL_DIAMETER             = 0.095;
    public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);

    //uses a trapezoidal velocity/time graph enforced with a PID loop
    private static ProfiledPIDController autoTurnPIDController
            = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(8, 8));

    static{
        autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI); //offset clamped between these two values
        autoTurnPIDController.setTolerance(Math.toRadians(10)); //tolerable error
    }

    // calculates target chassis motion when given current position and desired trajectory
    public static final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(0.35, 0, 0), // PID values for x offset
        new PIDController(0.35, 0, 0), // PID values for y offset
        autoTurnPIDController // PID values for orientation offset
    );
 }


  //-----------------------------Intake------------------------------------------

  public static final class IntakeConstants 
  {
    public static final double ROLLERS_PWR_CUBE_IN = -0.8;   
    public static final double ROLLERS_PWR_CONE_IN =  1.0; //TBD decide pwrs for all cube cone scoring rollers

    public static final double ROLLERS_PWR_CUBE_OUT =  1.0;   
    public static final double ROLLERS_PWR_CONE_OUT = -0.5;

    public static final double WRIST_MAX_PWR = 0.3;

    //----------------------------------------------------------------------------------------------
    //  Wrist encoder & Position Values
    //----------------------------------------------------------------------------------------------



    private static final double ENC_TO_INTAKE_GEAR_RATIO =  46.0/18.0 * 22.0/10.0;
    public static final double WRIST_CNTS_PER_DEGREE    = (2096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;


    public static final double MANUAL_HOLD_STEP_SIZE = 0.5;       

    //TBD - ADD comment for ref point
    public static final double CENTER_OF_MASS_OFFSET_DEG     = 177.0; 
    public static final double WRIST_ABS_ENC_OFFSET_DEG = 0.0; //Set to make stow pos equal to 0
    private static final double WRIST_ABS_ENC_OFFSET = WRIST_ABS_ENC_OFFSET_DEG * WRIST_CNTS_PER_DEGREE;//-989.0; //Negative value means abs enc 0 is above intake angle 0   
    
    public static final double STOW_DEG_POS               =  -20.0 + WRIST_ABS_ENC_OFFSET_DEG;//4872.0 + WRIST_ABS_ENC_OFFSET; //3883
    public static final double STOW_CUTOFF_DEG                =  -30.232 + WRIST_ABS_ENC_OFFSET_DEG;// + WRIST_ABS_ENC_OFFSET; //3670

    public static final double INTAKE_CUBE_DEG_POS        =  -140.000 + WRIST_ABS_ENC_OFFSET_DEG;//1324.0 + WRIST_ABS_ENC_OFFSET;    //-335
    public static final double INTAKE_CONE_DEG_POS_GROUND =  -150.524 + WRIST_ABS_ENC_OFFSET_DEG;//-306.0  + WRIST_ABS_ENC_OFFSET;  //-1295  
    public static final double INTAKE_CONE_DEG_POS_SINGLE =  -100.400 + WRIST_ABS_ENC_OFFSET_DEG;//2089.0 + WRIST_ABS_ENC_OFFSET;  //1100

    public static final double SCORE_CUBE_DEG_POS         =  -80.000 + WRIST_ABS_ENC_OFFSET_DEG;//1859.0 + WRIST_ABS_ENC_OFFSET;  //870     // Applies to low-mid-high

    public static final double SCORE_CONE_HIGH_DEG_POS    =  -153.000 + WRIST_ABS_ENC_OFFSET_DEG;//289.0 + WRIST_ABS_ENC_OFFSET;  //-700
    public static final double SCORE_CONE_MID_DEG_POS     = INTAKE_CONE_DEG_POS_GROUND; //TBD verify if its the same as high
    public static final double SCORE_CONE_LOW_DEG_POS     = INTAKE_CONE_DEG_POS_GROUND; //TBD


    public static  final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
    public static  final double SOFT_LIMIT_REVERSE = -5850.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD

    public static  final double GROSS_kP = 0.005;//0.0070;//0.00009; 
    public static  final double GROSS_kI = 0.0;//000040;
    public static  final double GROSS_kD = 0.0002;//0.000007;

    public static  final double FINE_kP = 0.003;//0.00009; 
    public static  final double FINE_kI = 0.0;//000008;
    public static  final double FINE_kD = 0.0007;//0.000291;//0.000007;
    
    public static  final double MAX_GRAVITY_FF = 0.09; //0.055


    public static final double INTAKE_POS_ERROR_THRESHOLD_ENC = 400;
    public static final double PID_FINE_GROSS_THRESHOLD_ENC = 700.0;
  }


//-------------------------------Elevator-------------------------------------------------------

public static final class ElevatorConstants
{

    public final double POS_ENC_CNTS_HIGH_EXTEND_THRESHOLD_ELEVATOR = 73000.0;


   /*********************************************8
    * 
    * Elevator manual Pwr Constants
    * 
    ****************************************/
    public static final double ELEVATOR_MAX_MANUAL_SCALED_POWER = 0.7;
    public static final double ELEVATOR_MANUAL_CONTROL_DEADBAND = 0.1;
    public static final double ELEVATOR_MANUAL_CONTROL_PWR_OFF = 0.0;
    public static final double ELEVATOR_MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;


    //constants for calc encoder to inch

    private static final double ELEVATOR_FIRST_GEAR1 = 13;
    private static final double ELEVATOR_FIRST_GEAR2 = 48;
    private static final double ELEVATOR_FIRST_GEAR_RATIO = ELEVATOR_FIRST_GEAR2/ELEVATOR_FIRST_GEAR1;

    private static final double ELEVATOR_HTD1 = 15;
    private static final double ELEVATOR_HTD2 = 30;
    private static final double ELEVATOR_HTD_RATIO = ELEVATOR_HTD2/ELEVATOR_HTD1;

    private static final double ELEVATOR_SECOND_GEAR1 = 28;
    private static final double ELEVATOR_SECOND_GEAR2 = 24;
    private static final double ELEVATOR_SECOND_GEAR_RATIO = ELEVATOR_SECOND_GEAR2/ELEVATOR_SECOND_GEAR1;

    private static final double ELEVATOR_FINAL_RATIO = ELEVATOR_FIRST_GEAR_RATIO*ELEVATOR_HTD_RATIO*ELEVATOR_SECOND_GEAR_RATIO;

    private static final double ELEVATOR_CNTS_TO_REV = 2048/1;

    private static final double ELEVATOR_SPROKET_DIAMETER = 1.751;
    private static final double ELEVATOR_SPROKET_CIRCUMFERENCE = ELEVATOR_SPROKET_DIAMETER*Math.PI;

    private static final double ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR = ((ELEVATOR_CNTS_TO_REV*ELEVATOR_FINAL_RATIO)/ELEVATOR_SPROKET_CIRCUMFERENCE);


    //----------------------------------------------------------------------------------------------
    //  Elevator Position Values
    //----------------------------------------------------------------------------------------------

    public static final double ELEVATOR_POS_ENC_INCH_LOW  = 0.0;
    public static final double ELEVATOR_POS_ENC_INCH_MID_CONE  = 39.616;
    public static final double ELEVATOR_POS_ENC_INCH_MID_CUBE  = 26;
    public static final double ELEVATOR_POS_ENC_INCH_HIGH = 47.187;

    public static final double ELEVATOR_POS_ENC_CNTS_LOW  = ELEVATOR_POS_ENC_INCH_LOW * ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR;
    public static final double ELEVATOR_POS_ENC_CNTS_MID_CONE  = ELEVATOR_POS_ENC_INCH_MID_CONE * ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR;//91000.0;// needs to be lower...too high
    public static final double ELEVATOR_POS_ENC_CNTS_MID_CUBE  = ELEVATOR_POS_ENC_INCH_MID_CUBE * ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR;
    public static final double ELEVATOR_POS_ENC_CNTS_SINGLE_PICKUP = 27739;

    public static final double ELEVATOR_POS_ENC_CNTS_HIGH = ELEVATOR_POS_ENC_INCH_HIGH * ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR;//111200.0;

    public static final double ELEVATOR_KP_LOW = 0.03;
    public static final double ELEVATOR_KI_LOW = 0.0002;
    public static final double ELEVATOR_KD_LOW = 0.001;

    public static final double ELEVATOR_KP_MID = 0.078;//lc
    public static final double ELEVATOR_KI_MID = 0.0002;
    public static final double ELEVATOR_KD_MID = 0.001;

    public static final double ELEVATOR_KP_HIGH = ELEVATOR_KP_MID;
    public static final double ELEVATOR_KI_HIGH = ELEVATOR_KI_MID;
    public static final double ELEVATOR_KD_HIGH = ELEVATOR_KD_MID;

    public static final double ELEVATOR_ARM_ENCODER_THRESHOLD = 35000.0;

    public static final double ELEVATOR_HOLDING_FF = 0.044;


    public static final double ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_LOW = 50; 

    public static final double ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID = 225; //lc


    public static final double ELEVATOR_POS_ERROR_THRESHOLD = 1000.0; //0.424 inches

    public static final double NO_TARGET_POSITION = -999999.0;
}


    //-----------------------------------ARM---------------------------------------
    public static final class ArmConstants
    {
      public static final double EXTEND_PWR  = 0.2;
      public static final double RETRACT_PWR = -0.2;
  
      //gear ratio
      private static final double VERSA_RATIO  = 7.0/1.0;
  
      private static final double PUILEY_1      = 24.0;
      private static final double PUILEY_2      = 18.0;
      private static final double PUILEY_RATIO  = PUILEY_1 / PUILEY_2;
       
      private static final double FINAL_RATIO   = VERSA_RATIO * PUILEY_RATIO;
      public static final double FINAL_CIRCUMFERENCE = 3.54; 
  
  
      public static final boolean LIMIT_SWITCH_IGNORED = false;
      public static final boolean LIMIT_SWITCH_MONITORED = true;
  
      public static final double CNTS_OVER_REV = 2048.0 / 1.0;
  
      public static final double CNTS_PER_INCH_CONVERSION_FACTOR = CNTS_OVER_REV/FINAL_CIRCUMFERENCE;
  
      public static final double POS_ENC_INCH_RETRACT = 0.0;
      public static final double POS_ENC_INCH_EXTEND = 8.157;
      public static final double POS_ENC_INCH_PICKUP = 4.157;
  
      public static final double POS_ENC_CNTS_RETRACT  = 10 * POS_ENC_INCH_RETRACT * CNTS_PER_INCH_CONVERSION_FACTOR;
      public static final double POS_ENC_CNTS_EXTEND  = 10 * POS_ENC_INCH_EXTEND * CNTS_PER_INCH_CONVERSION_FACTOR; //TBD we should continue with cnts per inch 
      public static final double POS_ENC_CNTS_PICKUP = 10 * POS_ENC_INCH_PICKUP * CNTS_PER_INCH_CONVERSION_FACTOR;
  
      public static final double POS_ENC_CNTS_HIGH_EXTEND_THRESHOLD_ELEVATOR = 73000.0;
  
  
  
      public static final double MANUAL_CONTROL_PWR_OFF = 0.0;

  
      public static final double ARM_POS_ERROR_THRESHOLD = 2700.0; //0.5 inches    previously 500 enc counts //TBD what does this mean
      public static final double NO_TARGET_POSITION = -999999.0;
    }
  
}
