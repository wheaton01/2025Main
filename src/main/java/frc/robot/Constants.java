// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
/* 
╔══════════════════════════════════════════════════════════════════════════════════════╗
║  __/\\\\\\\\\\\\\\\___/\\\\\\\\\\\\\\\____/\\\\\\\\\\\\\\\______/\\\\\\\\\\_________ ║
║  _\/////////////\\\__\/\\\///////////____\/\\\///////////_____/\\\///////\\\________ ║
║   ____________/\\\/___\/\\\_______________\/\\\_______________\///______/\\\________ ║
║   __________/\\\/_____\/\\\\\\\\\\\\______\/\\\\\\\\\\\\_____________/\\\//_________ ║
║    ________/\\\/_______\////////////\\\____\////////////\\\__________\////\\\_______ ║
║     ______/\\\/____________________\//\\\______________\//\\\____________\//\\\_____ ║
║      ____/\\\/___________/\\\________\/\\\___/\\\________\/\\\___/\\\______/\\\_____ ║
║       __/\\\/____________\//\\\\\\\\\\\\\/___\//\\\\\\\\\\\\\/___\///\\\\\\\\\/_____ ║
║        _\///_______________\/////////////______\/////////////_______\/////////______ ║
╚══════════════════════════════════════════════════════════════════════════════════════╝ 
╔══════════════════════════════════════════════════════════════════════════════════════╗
║                             ROBOT CONSTANTS CONFIGURATION                            ║
╚══════════════════════════════════════════════════════════════════════════════════════╝
*/

import static edu.wpi.first.units.Units.Newton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swervedrive.ApriltagRelativeRobotPose;

public final class Constants {

    //======================================================================================
    // Operator Constants
    //======================================================================================
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    //======================================================================================
    // Robot Constants - Elevator, Intake, and Climber Configuration
    //======================================================================================
    public static class robotConstants {
        
        // Elevator Spark IDs (Motor Controllers)
        public static final int kelevatorSparkID1 = 10; 
        public static final int kelevatorSparkID2 = 11; 
        
        // End Effector Constants (Intake, Placement, Pneumatics)
        public static final int kintakeSparkID = 12;   
        public static final int kPlacesparkID = 13;    
        public static final int kPneuExtendID = 7;       // Pneumatic extend ID
        
        // Intake Sensor ID and Threshold
        public static int kIntakeSensorID = 0;        
        public static double kintakeSensorThreshold = 2000.0;
        public static int elevatorHomingSWID = 0;

        //vision Constants
        public static final int ilCameraID = 0;//camera ID for the left camera
        public static final int irCameraID = 1;//camera ID for the right camera
        public static final int icCameraID = 2;//camera ID for the center camera

        //==================================================================================
        // Climber Constants
        //==================================================================================
        public static class climberConstants {
            public static final int kClimb2ID = 6;
            public static final int kClimb1ID = 4;
            public static final int kRamp1ID = 5;
            public static final int kMotorID = 18;    
            public static final double kClimbSpeed = -1.0;//Inverted 3/21
        }

        //==================================================================================
        // Elevator Constants
        //==================================================================================
        public static class elevatorConstants {
            public static boolean btestMode = false;
            public static boolean btwoMotorMode = false;

            // PID Constants for Upward and Downward Movement
            public static double kP_down = 0.0035;  // Slightly higher P for more correction towards target
            public static double kI_down = 0.0002;  // Small increase in I to help with steady-state error
            public static double kD_down = 0.00003; // Keep D low to avoid jerky motion
            
            public static double kFeedForwardDown = 0.075;  // Slight increase for better gravity compensation
            public static final double kFeedForward = 0.0750; // Tuned in 2/20
            public static double kP_up = 0.03;
            public static double kI =    0.00015;
            public static double kD =    0.00005;

            // PID Zone and Threshold Constants
            public static double kIZone = 5.0;
            public static double kdownSpeed = -0.7;
            public static double kPIDThreshold = 2.0;

            // Elevator Height Constants
            public static final double kL4Height = 950.0;
            public static final double kL3Height = 580.0; 
            public static final double kL2Height = 390.0; 
            public static final double kL1Height = 0.0;

            public static double kProcessorhHeight = 120;


            // Home and Max Height Constants
            public static final double kHomePose = 0.0;
            public static final double kMaxHeight = 1000.0;
            public static final double kCURRENT_THRESHOLD = 4.0;//used for homing

            // Encoder Constants
            public static final double kTolerance = 1.0;
            public static final int kEncoderA = 0;
            public static final int kEncoderB = 1;
            public static final double kElevatorConversionFac = 8.665511; // Conversion factor for encoder ticks to inches
        }

        //==================================================================================
        // AprilTag Constants (Offsets)
        //==================================================================================
        public static class aprilTagConstants {
            public static double klReefOffset = -5.0;
            public static double krReefOffset = 5.0;
        }

        //==================================================================================
        // Intake Constants
        //==================================================================================
        public static class intakeConstants {
            public static final double kIdleIntakeSpeed = 1.0;
            public static final double kIntakeSpeed = 1.0;
            public static final double kBallIntakeSpeed = -1.0;
            public static final double kPlaceSpeed = 1.0;
            public static final double kIntakeSensorThreshold = 0.0;
            public static final double kIntakeTime = 0.0;
            public static final double kL4PlaceTime = 0.25;  // Delay for intake retraction
            public static final double kPlaceTime = 0.25;    // Delay for intake stop after placement
        }
    }

    //======================================================================================
    // Swerve Drive Constants
    //======================================================================================
    public static class swerveConstants {
        public static final double kalignSpeed = 0.4;
        public static final double MAX_SPEED = 5.09;
        public static double kforwardOffsetDistance = 1.460; // Distance for drive-to-pose
        public static double sideOffsetDistance = 0.165;     // Distance between robot center and AprilTag

        // Tolerance Constants for Position and Rotation
        public static final double POSITION_TOLERANCE = 0.5;  // Tolerance for position accuracy
        public static final double ROTATION_TOLERANCE = 0.5;  // Tolerance for rotation accuracy
        public static final PIDController SWERVEX_CONTROLLER = new PIDController(1.0, 0.0, 0.0);
        public static final PIDController SWERVEY_CONTROLLER = new PIDController(1.0, 0.0, 0.0);
        public static final PIDController SWERVETHETA_CONTROLLER = new PIDController(0.03, 0.0, 0.0);
        
        public static final double xOffsetReef = .3 ;
        public static final double yOffsetReef = 0.25;
    }
    public static class fieldPoses {
        public static final Pose2d reefPose = new Pose2d(new Translation2d(5.034, 5.317), Rotation2d.fromDegrees(117));
        public static final ApriltagRelativeRobotPose lSidePose = new ApriltagRelativeRobotPose(0.0, 0.0, 0.0);
        public static final ApriltagRelativeRobotPose rSidePose = new ApriltagRelativeRobotPose(0.0, 0.0, 0.0);
        public static final ApriltagRelativeRobotPose centerPose = new ApriltagRelativeRobotPose(0.0, 0.0, 0.0);
    }
    public static class poseConstants{
        public static final double xOffsetHPStation = Units.inchesToMeters(12.0) ;
        public static final double yOffsetHPStation = Units.inchesToMeters(7.5) ;

    }
}
