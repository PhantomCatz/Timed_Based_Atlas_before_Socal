/***
 * CatzConstants
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is where reusable constants are defined
 * -TBD should all constants move here?
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

 //Saving for finished trajectory planner
  //----------------------Catz auton Constants---------------------------
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

    public static final double MAX_SPEED = 3.0; // meters per second
    public static final double MAX_ANGSPEED = 6.0; // radians per second

    public static final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio reduction
    public static final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio reduction
    
    public static final double DRVTRAIN_WHEEL_DIAMETER             = 0.095;
    public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);

    //uses a trapezoidal velocity/time graph enforced with a PID loop
    private static ProfiledPIDController autoTurnPIDController
            = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGSPEED, MAX_ANGSPEED));

    static{
        autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI); //offset clamped between these two values
        autoTurnPIDController.setTolerance(Math.toRadians(0.1)); //tolerable error
    }

    // calculates target chassis motion when given current position and desired trajectory
    public static final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(0.15, 0, 0), // PID values for x offset
        new PIDController(0.15, 0, 0), // PID values for y offset
        autoTurnPIDController // PID values for orientation offset
    );
 }


  //-----------------------------Intake------------------------------------------

  public static final class IntakeConstants 
  {
    //----------------------------------------------------------------------------------------------
    //  Wrist encoder & Position Values
    //----------------------------------------------------------------------------------------------
    private static final int    INTAKE_WRIST_ENC_CAN_ID = 13; 


    private static final double INTAKE_ENC_TO_INTAKE_GEAR_RATIO =  46.0/18.0;
    public static final double INTAKE_WRIST_CNTS_PER_DEGREE    = 46.459; //(4096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;


    public static final double INTAKE_MANUAL_HOLD_STEP_SIZE = 1.5;       

    //TBD - ADD comment for ref point
    //Reference Point = wrist would be slight above "Parallel to the ground"
    public static final double INTAKE_CENTER_OF_MASS_OFFSET_DEG     = 177.0; 
    public static final double INTAKE_WRIST_ABS_ENC_OFFSET_DEG = 0.0; //Set to make stow pos equal to 0
    public static final double INTAKE_WRIST_ABS_ENC_OFFSET = INTAKE_WRIST_ABS_ENC_OFFSET_DEG * INTAKE_WRIST_CNTS_PER_DEGREE;//-989.0; //Negative value means abs enc 0 is above intake angle 0   
    
    public static final double STOW_ENC_POS               =  0.0 + INTAKE_WRIST_ABS_ENC_OFFSET_DEG;//4872.0 + WRIST_ABS_ENC_OFFSET; //3883
    public static final double STOW_CUTOFF                =  -7.232 + INTAKE_WRIST_ABS_ENC_OFFSET_DEG;// + WRIST_ABS_ENC_OFFSET; //3670

    public static final double INTAKE_CUBE_ENC_POS        =  -147.000 + INTAKE_WRIST_ABS_ENC_OFFSET_DEG;//1324.0 + WRIST_ABS_ENC_OFFSET;    //-335
    public static final double INTAKE_PICKUP_CONE_ENC_POS_GROUND =  -184.524 + INTAKE_WRIST_ABS_ENC_OFFSET_DEG;//-306.0  + WRIST_ABS_ENC_OFFSET;  //-1295  
    public static final double INTAKE_PICKUP_CONE_ENC_POS_SINGLE =  -116.400 + INTAKE_WRIST_ABS_ENC_OFFSET_DEG;//2089.0 + WRIST_ABS_ENC_OFFSET;  //1100

    public static final double SCORE_CUBE_ENC_POS         =  -104.000 + INTAKE_WRIST_ABS_ENC_OFFSET_DEG;//1859.0 + WRIST_ABS_ENC_OFFSET;  //870     // Applies to low-mid-high

    public static final double SCORE_CONE_HIGH_ENC_POS    =  -153.000 + INTAKE_WRIST_ABS_ENC_OFFSET_DEG;//289.0 + WRIST_ABS_ENC_OFFSET;  //-700
    public static final double SCORE_CONE_MID_ENC_POS     = INTAKE_PICKUP_CONE_ENC_POS_GROUND; //TBD verify if its the same as high
    public static final double SCORE_CONE_LOW_ENC_POS     = INTAKE_PICKUP_CONE_ENC_POS_GROUND; //TBD


    public static final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
    public static final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD

    public static final double GROSS_kP = 0.002472;//0.00009; 
    public static final double GROSS_kI = 0.0;//000040;
    public static final double GROSS_kD = 0.000291;//0.000007;

    public static final double FINE_kP = 0.005234;//0.00009; 
    public static final double FINE_kI = 0.0;//000008;
    public static final double FINE_kD = 0.000291;//0.000007;
    
    public static final double MAX_GRAVITY_FF = 0.055; //0.09\

  }


//-------------------------------Elevator-------------------------------------------------------

public static final class ElevatorConstants
{

    public final double POS_ENC_CNTS_HIGH_EXTEND_THRESHOLD_ELEVATOR = 73000.0;
    public final static double ELEVATOR_MAX_MANUAL_SCALED_POWER = 0.7;

    public final double ELEVATOR_MANUAL_CONTROL_DEADBAND = 0.1;

    public final double ELEVATOR_MANUAL_CONTROL_PWR_OFF = 0.0;

    public final double ELEVATOR_MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;


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
    // private final double POS_ENC_CNTS_MID_CUBE  = POS_ENC_INCH_MID_CUBE * INCHES_TO_COUNTS_CONVERSTION_FACTOR;
    public static final double ELEVATOR_POS_ENC_CNTS_MID_CUBE = 50000.0;
    public static final double ELEVATOR_POS_ENC_CNTS_SINGLE_PICKUP = 27739;

    public static final double ELEVATOR_POS_ENC_CNTS_HIGH = ELEVATOR_POS_ENC_INCH_HIGH * ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR;//111200.0;

    public static final double ELEVATOR_KP_LOW = 0.03;
    public static final double ELEVATOR_KI_LOW = 0.0002;
    public static final double ELEVATOR_KD_LOW = 0.001;

    public static final double ELEVATOR_KP_MID = 0.083;
    public static final double ELEVATOR_KI_MID = 0.0002;
    public static final double ELEVATOR_KD_MID = 0.0;

    public static final double ELEVATOR_KP_HIGH = ELEVATOR_KP_MID;
    public static final double ELEVATOR_KI_HIGH = ELEVATOR_KI_MID;
    public static final double ELEVATOR_KD_HIGH = ELEVATOR_KD_MID;

    public static final double ELEVATOR_ARM_ENCODER_THRESHOLD = 35000.0;

    public static final double ELEVATOR_HOLDING_FF = 0.044;


    public static final double ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_LOW = 50; 
    // private final double CLOSELOOP_ERROR_THRESHOLD_HIGH_MID = 300; 
    public static final double ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID = 225; 
}


    //-----------------------------------ARM---------------------------------------
    public static final class ArmConstants
    {
    //gear ratio
    private static final double ARM_VERSA_RATIO  = 7.0/1.0;

    private static final double PUILEY_1      = 24.0;
    private static final double PUILEY_2      = 18.0;
    private static final double PUILEY_RATIO  = PUILEY_1 / PUILEY_2;
      
    private static final double ARM_FINAL_RATIO   = ARM_VERSA_RATIO * PUILEY_RATIO;
    private static final double FINAL_CIRCUMFERENCE = 3.54; 

    private static final double CNTS_OVER_REV = 2048.0 / 1.0;

    private static final double CNTS_PER_INCH_CONVERSION_FACTOR = CNTS_OVER_REV/FINAL_CIRCUMFERENCE;
  
    private static final double POS_ENC_INCH_RETRACT = 0.0;
    private static final double POS_ENC_INCH_EXTEND = 8.157;
    private static final double POS_ENC_INCH_PICKUP = 4.157;
  
    public static final double POS_ENC_CNTS_RETRACT  = POS_ENC_INCH_RETRACT * CNTS_PER_INCH_CONVERSION_FACTOR;// 0.0
    public static final double POS_ENC_CNTS_EXTEND  = POS_ENC_INCH_EXTEND * CNTS_PER_INCH_CONVERSION_FACTOR; //44000
    public static final double POS_ENC_CNTS_PICKUP = POS_ENC_INCH_PICKUP * CNTS_PER_INCH_CONVERSION_FACTOR; //22000
    }
  
}
