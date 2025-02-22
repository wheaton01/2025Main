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
        public static final int kelevatorSparkID1 = 10; // TODO : CORRECT CAN ID
        public static final int kelevatorSparkID2 = 11; // TODO : CORRECT CAN ID
        
        // End Effector Constants (Intake, Placement, Pneumatics)
        public static final int kintakeSparkID = 13;    // TODO : CORRECT CAN ID
        public static final int kPlacesparkID = 12;     // TODO : CORRECT CAN ID
        public static final int kPneuExtendID = 7;       // Pneumatic extend ID
        
        // Intake Sensor ID and Threshold
        public static int kIntakeSensorID = 0;          // TODO : CORRECT ANALOG ID
        public static double kintakeSensorThreshold = 1200;

        //==================================================================================
        // Climber Constants
        //==================================================================================
        public static class climberConstants {
            public static final int kClimb2ID = 6;
            public static final int kClimb1ID = 4;
            public static final int kRamp1ID = 3;    
        }

        //==================================================================================
        // Elevator Constants
        //==================================================================================
        public static class elevatorConstants {
            public static boolean btestMode = true;
            public static boolean btwoMotorMode = false;

            // PID Constants for Upward and Downward Movement
            public static double kP_down = 0.0025;  // Slightly higher P for more correction towards target
            public static double kI_down = 0.0002;  // Small increase in I to help with steady-state error
            public static double kD_down = 0.00005; // Keep D low to avoid jerky motion
            
            public static double kFeedForwardDown = 0.12;  // Slight increase for better gravity compensation
            public static final double kFeedForward = 0.0750; // Tuned in 2/20
            public static double kP_up = 0.002;
            public static double kI = 0.00015;
            public static double kD = 0.0005;

            // PID Zone and Threshold Constants
            public static double kIZone = 5.0;
            public static double kdownSpeed = -0.4;
            public static double kPIDThreshold = 2.0;

            // Elevator Height Constants
            public static final double kL4Height = 1450.0;
            public static final double kL3Height = 880.0; 
            public static final double kL2Height = 520.0; 
            public static final double kL1Height = 0.0;

            // Home and Max Height Constants
            public static final double kHomePose = 0.0;
            public static final double kMaxHeight = 2000.0;

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
            public static final double kIntakeTime = 0.1;
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
        public static double kforwardOffsetDistance = 0.460; // Distance for drive-to-pose
        public static double sideOffsetDistance = 0.165;     // Distance between robot center and AprilTag

        // Tolerance Constants for Position and Rotation
        public static final double POSITION_TOLERANCE = 0.5;  // Tolerance for position accuracy
        public static final double ROTATION_TOLERANCE = 0.5;  // Tolerance for rotation accuracy
    }
}
