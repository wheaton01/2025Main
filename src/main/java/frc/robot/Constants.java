// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  public static class robotConstants{
    public static final int kelevatorSparkID1 = 10;//TODO : CORRECT CAN ID
    public static final int kelevatorSparkID2 = 11;//TODO : CORRECT CAN ID
    
    //End Affector Constants
      public static final int kintakeSparkID = 13;//TODO : CORRECT CAN ID
      public static final int kPlacesparkID = 12;//TODO : CORRECT CAN ID

      public static final int kPneuExtendID = 7;
      public static final int kClimb2ID = 6;
      public static final int kClimb1ID = 4;
      public static final int kRamp1ID = 3;


      public static int kIntakeSensorID = 0; //TODO : CORRECT ANALOG ID
      //public static int kPlaceSensorID  = 0;      //TODO : CORRECT ANALOG ID


      public static double kintakeSensorThreshold = 1200;


    public static class climberConstants{
      public static final int kClimberSolenoidID = 4;//TODO : CORRECT PNEUMATIC ID
      public static final int kDeployClimberSolenoidID = 5;//TODO : CORRECT PNEUMATIC ID
    }

      public static class elevatorConstants{

        public static final double kFeedForward = .080;//tuned in 2/20
        public static double kP_up = 0.002;
        public static double kP_down = .0005;
        //public  static double kP = 0.00015;
        public  static double kI = 0.00015;
        public  static double kD = 0.00015;

        public static double kIZone = 5.0;

        public static final double kL4Height = 750.0;
        public static final double kL3Height = 500.0; 
        public static final double kL2Height = 250.0; 
        public static final double kL1Height = 0.0;

        public static final double kHomePose = 0.0;

        public static final double kMaxHeight = 1000.0;

        public static final double kTolerance = 1.0;
        public static final int    kEncoderA  = 0;
        public static final int    kEncoderB  = 1;
        public static final double kElevatorConversionFac = 0.18838;//TODO : This value is used to convert the encoder ticks to inches. this value is pulled from cad and may need to be corrected based on field meas.

      }
      public static class aprilTagConstants{

        public static double klReefOffset= -5.0;
        public static double krReefOffset= 5.0;

      }
      public static class intakeConstants{
        public static final double kIdleIntakeSpeed = .5;
        public static final double kIntakeSpeed = 0.7;
        public static final double kBallIntakeSpeed = -1.0;
        public static final double kPlaceSpeed = 1.0;
        public static final double kIntakeSensorThreshold = 0.0;
        public static final double kIntakeTime = .1;
        public static final double kL4PlaceTime  = .25;//TODO : This value is used in the placeL4 to set a delay so the intake dosent retract too quickly
        public static final double kPlaceTime  = .25;//TODO : This value is used in the placeCoral to set a delay the intake doenst turn off too quickly
      }

  }


  public static class swerveConstants{
    public static final double kalignSpeed = .4;
    public static final double MAX_SPEED = 5.09;
    public static double kforwardOffsetDistance = .460;//TODO : This value is pulled from cad and may need to be corrected based on field meas. this is used for the drivetoPose command
    public static double sideOffsetDistance     = .165;//TODO : Sets distance between the center of the robot and april tag

    public static final double POSITION_TOLERANCE = 0.5;//TODO : this value is used in the swerveSubsystem to determine how close the robot is to the target pose
    public static final double ROTATION_TOLERANCE = 0.5;
    
  }

    
    
  
}
