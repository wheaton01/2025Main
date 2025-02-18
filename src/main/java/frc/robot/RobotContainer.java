package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.robotConstants.elevatorConstants;
import frc.robot.Constants.robotConstants.intakeConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.setCHaptics;
import frc.robot.commands.climberCommands.setClimber;
import frc.robot.commands.elevatorCommands.setElevatorPose;
import frc.robot.commands.intakeCommands.setIntake;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.subsystems.sClimber;
import frc.robot.subsystems.sControllerHaptics;
import frc.robot.subsystems.sElevator;
import frc.robot.subsystems.sEndAffector;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    sEndAffector sEndAffector;
    sClimber sClimber;
    sElevator sElevator;
    SwerveSubsystem swerveSubsystem;

    setIntake defaultIntake, intakeBall;
    setClimber defaultClimber;
    sControllerHaptics m_controllerHaptics;
    setElevatorPose setL4Pose, setL3Pose, setL2Pose, setL1Pose, setHomePose;
    DriveToPoseCommand autoDriveToPoseCommand;

    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    public final CommandXboxController m_operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);

    public RobotContainer() {
        //sEndAffector = new sEndAffector();
        //sClimber = new sClimber();
        //sElevator = new sElevator();
       // m_controllerHaptics = new sControllerHaptics(m_driverController, m_operatorController);

        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                "neo"));
        setupCommands();
        configureBindings();
    }

    private void setupCommands() {
        // // Intake Commands
        // defaultIntake = new setIntake(false, false, 0, 0, sEndAffector);
        // defaultClimber = new setClimber(sClimber, false, false);
        // intakeBall = new setIntake(false, true, 0.0, -1.0, sEndAffector);

        // // Elevator Commands
        // setL4Pose = new setElevatorPose(sElevator, elevatorConstants.kL4Height);
        // setL3Pose = new setElevatorPose(sElevator, elevatorConstants.kL3Height);
        // setL2Pose = new setElevatorPose(sElevator, elevatorConstants.kL2Height);
        // setL1Pose = new setElevatorPose(sElevator, elevatorConstants.kL1Height);
        // setHomePose = new setElevatorPose(sElevator, elevatorConstants.kHomePose);

        // // Setting Default Commands
        // sEndAffector.setDefaultCommand(defaultIntake);
        // sClimber.setDefaultCommand(defaultClimber);

        //swerve Commands
        autoDriveToPoseCommand = new  DriveToPoseCommand(swerveSubsystem,swerveConstants.kalignSpeed,()->0.0,
        ()->0.0, 
        ()->0.0,
        ()->0.0, 
        ()->0.0,
        false, 
        true);
    }

    private void configureBindings() {
        driverControls();
        //operatorControls();
    }

    public void driverControls() {
        
        // Set default drive command
        swerveSubsystem.setDefaultCommand(
                swerveSubsystem.driveCommand(
                        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1),
                        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
                        () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.1)
                )
        );
        
        m_driverController.a().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
        m_driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyroWithAlliance));

        // Create DriveToPoseCommand based on trigger inputs
        createDriveToPoseTrigger(m_driverController::getRightTriggerAxis, true);
        createDriveToPoseTrigger(m_driverController::getLeftTriggerAxis, false);
                
        // Bind B button to drive to nearest AprilTag pose at center
        createDriveToPoseButtonTrigger(m_driverController.b(), false);

        // Bind right bumper to drive to nearest AprilTag pose with a special mode
        createDriveToPoseButtonTrigger(m_driverController.rightBumper(), true);
    }

    // Helper method to create drive-to-pose triggers for the right and left
    // triggersprivate void createDriveToPoseTrigger(DoubleSupplier triggerSupplier, boolean isRightTrigger) {
private void createDriveToPoseTrigger(DoubleSupplier triggerSupplier, boolean isRightTrigger) {
    
        new Trigger(() -> triggerSupplier.getAsDouble() > 0.5)
            .whileTrue(new DriveToPoseCommand(
                    swerveSubsystem,
                    swerveConstants.MAX_SPEED,
                    m_driverController::getLeftX,
                    m_driverController::getLeftY,
                    m_driverController::getRightX,
                    m_driverController::getLeftTriggerAxis,
                    m_driverController::getRightTriggerAxis,
                    !isRightTrigger,
                    false
            ));
}



    // Helper method to bind a button to a DriveToPoseCommand
    private void createDriveToPoseButtonTrigger(Trigger button, boolean specialMode) {
        button.whileTrue(new DriveToPoseCommand(
                swerveSubsystem,
                swerveConstants.MAX_SPEED,
                m_driverController::getLeftX,
                m_driverController::getLeftY,
                m_driverController::getRightX,
                m_driverController::getLeftTriggerAxis,
                m_driverController::getRightTriggerAxis,
                specialMode,
                false
        ));
    }
    
    public void operatorControls() {
        // ------------------------- Preset Pose Commands ------------------------- //
        m_operatorController.a().onTrue(setL1Pose);
        m_operatorController.b().onTrue(setL3Pose);
        m_operatorController.x().onTrue(setL2Pose);
        m_operatorController.a().onTrue(setHomePose); // Note: A button duplicated
    
        // ----------------------- Climber Safety Override ----------------------- //
        new Trigger(() -> m_operatorController.rightStick().getAsBoolean() &&
                m_operatorController.leftStick().getAsBoolean())
                .onTrue(new ParallelCommandGroup(
                        new setCHaptics(m_controllerHaptics, 0.8).withTimeout(1.2),
                        new InstantCommand(sClimber::disableSafety)));
    
        // --------------------------- Intake Controls --------------------------- //
    
        // **Left Bumper**: Deploy intake only (no motor action) -> No haptic feedback when motors are off
        m_operatorController.leftBumper()
                .onTrue(new setIntake(false, true, 0, 0, sEndAffector)); // No haptics here since motors are off
    
        // **Left Trigger (≥ 0.2)**: Deploy & run **intake forward, place motor in reverse**
        m_operatorController.leftTrigger(0.2)
                .whileTrue(new ParallelCommandGroup(
                        new setIntake(false, true, intakeConstants.kIntakeSpeed, -intakeConstants.kPlaceSpeed, sEndAffector),
                        new setCHaptics(m_controllerHaptics, 0.5))); // Haptic feedback when motors are on
    
        // **Right Trigger (≥ 0.2)**: Deploy & run **both intake & place motors forward**
        m_operatorController.rightTrigger(0.2)
                .whileTrue(new ParallelCommandGroup(
                        new setIntake(false, true, intakeConstants.kIntakeSpeed, intakeConstants.kPlaceSpeed, sEndAffector),
                        new setCHaptics(m_controllerHaptics, 0.7))); // Haptic feedback when motors are on
    
        // **Right Bumper**: Auto-Intake Mode (Both Motors Forward)
        m_operatorController.rightBumper()
                .whileTrue(new ParallelCommandGroup(
                        new setIntake(true, true, intakeConstants.kIntakeSpeed, intakeConstants.kPlaceSpeed, sEndAffector),
                        new setCHaptics(m_controllerHaptics, 0.8))); // Haptic feedback when motors are on
    
        // **Home Position (Idle Intake Mode)**: Runs intake at low speed until a note is detected
        new Trigger(() -> !m_operatorController.leftTrigger(0.2).getAsBoolean() &&
                !m_operatorController.rightTrigger(0.2).getAsBoolean() &&
                !m_operatorController.leftBumper().getAsBoolean() &&
                !m_operatorController.rightBumper().getAsBoolean())
                .whileTrue(new setIntake(false, false, intakeConstants.kIdleIntakeSpeed, 0, sEndAffector)
                        .until(sEndAffector::getCoralSensor)); // Stops when note is detected
    }
    

    public Command getAutonomousCommand() {
        return Autos.getAutonomousCommand(sEndAffector);
    }
}