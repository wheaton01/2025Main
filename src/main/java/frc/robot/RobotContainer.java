package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.fieldPoses;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.robotConstants.elevatorConstants;
import frc.robot.commands.setCHaptics;
import frc.robot.commands.climberCommands.setClimber;
import frc.robot.commands.elevatorCommands.setElevatorOffset;
import frc.robot.commands.elevatorCommands.setElevatorPose;
import frc.robot.commands.intakeCommands.setIntake;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.commands.SwerveCommands.driveToPose;
import frc.robot.subsystems.sClimber;
import frc.robot.subsystems.sControllerHaptics;
import frc.robot.subsystems.sElevator;
import frc.robot.subsystems.sEndAffector;
import frc.robot.subsystems.sIntake;
import frc.robot.subsystems.sSlider;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.time.Instant;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
╔════════════════════════════════════════════════════════════════════════════════════════════════╗
║                         RobotContainer Class  ::  This is like our main class                  ║
║                                                 creates button binds and commands              ║ 
╚════════════════════════════════════════════════════════════════════════════════════════════════╝
*/
public class RobotContainer {
    sEndAffector sEndAffector;
    sSlider sSlider;
    sClimber sClimber;
    sElevator sElevator;
    SwerveSubsystem swerveSubsystem;
    sIntake sIntake;

    setIntake defaultIntake, intakeBall;
    setClimber defaultClimber;
    sControllerHaptics m_controllerHaptics;
    setElevatorPose setL4Pose, setL3Pose, setL2Pose, setL1Pose, setHomePose;
    DriveToPoseCommand autoDriveToPoseCommand;
    setElevatorOffset setElevatorOffset;
    Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);

    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    public final CommandXboxController m_operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);
         private final SendableChooser<Command> autoChooser;
    /* 
    ╔══════════════════════════════════════════════════════════════════════════════════════════╗
    ║                                RobotContainer Constructor                                ║
    ╚══════════════════════════════════════════════════════════════════════════════════════════╝
    */
    public setElevatorOffset opeartorOffset ;

    public RobotContainer() {

        sSlider = new sSlider();
        sEndAffector = new sEndAffector();
        sClimber = new sClimber();
        sElevator = new sElevator();
        m_controllerHaptics = new sControllerHaptics(m_driverController, m_operatorController);
        sIntake = new sIntake();
        registerNamedCommands();


        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                "neo"));
        setupCommands();
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("LsideStart");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        getAutonomousCommand();
    }

    /* 
    ╔════════════════════════════════════════════════════════════════════════════════════════════╗
    ║                                      Setup Commands                                        ║
    ╚════════════════════════════════════════════════════════════════════════════════════════════╝
    */
    private void setupCommands() {
       opeartorOffset = new setElevatorOffset(sElevator,()-> m_operatorController.getLeftY());
        // sElevator.setDefaultCommand(new SetManualElevator(
        //         sElevator,
        //         () -> 1.0 * MathUtil.applyDeadband(m_operatorController.getLeftY(), 0.2) 
        //     ));        // Intake Commands
        // defaultIntake = new setIntake(false, false, 0, 0, sEndAffector);
        defaultClimber = new setClimber(sClimber, false, false);
        //intakeBall = new setIntake(false, true, 0.0, -1.0, sEndAffector);

        // Elevator Commands
        setL4Pose = new setElevatorPose(sElevator, elevatorConstants.kL4Height);
        setL3Pose = new setElevatorPose(sElevator, elevatorConstants.kL3Height);
        setL2Pose = new setElevatorPose(sElevator, elevatorConstants.kL2Height);
        setL1Pose = new setElevatorPose(sElevator, elevatorConstants.kL1Height);
        setHomePose = new setElevatorPose(sElevator, elevatorConstants.kHomePose);

        //setElevatorOffset = new setElevatorOffset(sElevator, () -> MathUtil.applyDeadband(m_operatorController.getLeftY(), .1));

        // Setting Default Commands
        //sEndAffector.setDefaultCommand(defaultIntake);
        sClimber.setDefaultCommand(defaultClimber);

        //swerve Commands
        autoDriveToPoseCommand = new DriveToPoseCommand(swerveSubsystem, swerveConstants.kalignSpeed, () -> 0.0,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0,
                false,
                true);
    }

    /* 
    ╔══════════════════════════════════════════════════════════════════════════════════════════╗
    ║                                 Configure Controller Bindings                            ║
    ╚══════════════════════════════════════════════════════════════════════════════════════════╝
    */
    private void configureBindings() {
        setDefaultCommands();
        driverControls();
        operatorControls();
    }

    /* 
    ╔════════════════════════════════════════════════════════════════════════════════════════╗
    ║                              Driver Control Configuration                              ║
    ╚════════════════════════════════════════════════════════════════════════════════════════╝
    */
    public void driverControls() {
        
        // Set default drive command

        swerveSubsystem.setDefaultCommand(
                swerveSubsystem.driveCommand(
                        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1),
                        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1),
                        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1),
                        () -> MathUtil.applyDeadband(-m_driverController.getRightY(), 0.1)
                        
                )
        );
        m_driverController.leftBumper()
                 .whileTrue(
                         swerveSubsystem.driveCommand(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1)*.5,
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1)*.5,
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1)*.7));

        //sElevator.setDefaultCommand(sElevator.setElevator(() -> MathUtil.applyDeadband(m_operatorController.getLeftY(), .1)));
        m_driverController.a().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
        m_driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyroWithAlliance));
        //Driver Non Driving Controls
        m_driverController.rightBumper().onTrue(new ParallelCommandGroup(new InstantCommand(sIntake::setFeedIntake),
                                                                         new InstantCommand(sEndAffector::setPlace),
                                                                         new InstantCommand(sIntake::setCoralPlaceMode)))
                                          .onFalse(new ParallelCommandGroup(new InstantCommand(sIntake::setZero),
                                          new InstantCommand(sEndAffector::setZero),
                                          new InstantCommand(sIntake::setIntakeMode)).withTimeout(0));        
        // Create DriveToPoseCommand based on trigger inputs
        createDriveToPoseTrigger(m_driverController::getRightTriggerAxis, false);
        createDriveToPoseTrigger(m_driverController::getLeftTriggerAxis, false);
        
        // Bind B button to drive to nearest AprilTag pose at center
        //createDriveToPoseButtonTrigger(m_driverController.b(), false);

        // Bind right bumper to drive to nearest AprilTag pose with a special mode
        //createDriveToPoseButtonTrigger(m_driverController.rightBumper(), true);
        createDriveToPoseButtonTrigger(m_driverController.x(), true);
        m_driverController.b().whileTrue(new driveToPose(swerveSubsystem,fieldPoses.reefPose));
    }

    /* 
    ╔════════════════════════════════════════════════════════════════════════════════════════╗
    ║                                 Operator Control Configuration                         ║
    ╚════════════════════════════════════════════════════════════════════════════════════════╝
    */
    public void operatorControls() {
        
        // ------------------------- Preset Pose Commands ------------------------- //
        // m_operatorController.a().onTrue(setL1Pose);
        m_operatorController.rightBumper().onTrue(setL4Pose);
        m_operatorController.b().onTrue(setL3Pose);
        m_operatorController.x().onTrue(setL2Pose);
        m_operatorController.a().onTrue(setHomePose);
        // m_operatorController.a().and(m_operatorController.b().
        // and(m_operatorController.x().
        // and(m_operatorController.y().
        // and(m_operatorController.rightBumper().whileFalse(setElevatorOffset)))));
        
        // ----------------------- Climber Commands ----------------------- //
        new Trigger(() -> m_operatorController.rightStick().getAsBoolean() &&
                m_operatorController.leftStick().getAsBoolean())
                .onTrue(new SequentialCommandGroup(new ParallelCommandGroup(
                        new setCHaptics(m_controllerHaptics, 0.8).withTimeout(.75),
                        new InstantCommand(sClimber::disableSafety),
                        new InstantCommand(sClimber::dropRamp)),
                        new InstantCommand(sClimber::deployClimber))                        
                        );

        m_operatorController.povDown().onTrue(new ParallelCommandGroup(new InstantCommand(sClimber::climb),
                                                                       new InstantCommand(sClimber::stowClimber)))
                                                                       .onFalse(new InstantCommand(sClimber::zeroClimb));
        m_operatorController.povUp().onTrue(new ParallelCommandGroup(new InstantCommand(sClimber::unClimb),
                                                                     new InstantCommand(sClimber::deployClimber)))
                                                                     .onFalse(new InstantCommand(sClimber::zeroClimb));
        m_operatorController.povLeft().onTrue(new InstantCommand(sClimber::stowClimber));
        m_operatorController.povRight().onTrue(new InstantCommand(sClimber::deployClimber));
        // m_operatorController.start().onTrue(new InstantCommand(sClimber::dropRamp));
    
        // --------------------------- Intake Controls --------------------------- //
        m_operatorController.rightStick().onTrue(new InstantCommand(sIntake::hardResetIntake));
        // **Left Bumper**: Deploy intake only (no motor action) -> No haptic feedback when motors are off
        m_operatorController.leftBumper()
                .onTrue(new InstantCommand(sSlider::setExtend))
                .onFalse(new InstantCommand(sSlider::setRetract)); // No haptics here since motors are off

        // **Left Trigger (≥ 0.2)**: Deploy & run **intake forward, place motor in reverse**
        m_operatorController.leftTrigger(0.2)
                .whileTrue(new ParallelCommandGroup(
                        new InstantCommand(sEndAffector::setBallIntake),
                        new setCHaptics(m_controllerHaptics, 0.2))
                        ).onFalse(new InstantCommand(sEndAffector::setZero)); // Haptic feedback when motors are on
        m_operatorController.rightTrigger(0.2)
                        .whileTrue(new ParallelCommandGroup(
                                new InstantCommand(sEndAffector::setPlace),
                                new setCHaptics(m_controllerHaptics, 0.2))
                                ).onFalse(new InstantCommand(sEndAffector::setZero)); // Haptic feedback when motors are on
        }

    /* 
    ╔════════════════════════════════════════════════════════════════════════════════════════════╗
    ║                              Set Default Commands                                          ║
    ╚════════════════════════════════════════════════════════════════════════════════════════════╝
    */
    public void setDefaultCommands() {
        sElevator.setDefaultCommand(opeartorOffset);
        // if (elevatorConstants.btestMode) {
        //     sElevator.setDefaultCommand(sElevator.setElevator(() -> MathUtil.applyDeadband(m_operatorController.getLeftY(), .1)));
        // }
        //sElevator.setDefaultCommand(setElevatorOffset);

        // Automatically start intakeCoral when coral is NOT detected

        // sSlider.setDefaultCommand(new InstantCommand(sSlider::setRetract));
    }

    /* 
    ╔══════════════════════════════════════════════════════════════════════════════════════════╗
    ║                            Get Autonomous Command                                        ║
    ╚══════════════════════════════════════════════════════════════════════════════════════════╝
    */
    public Command getAutonomousCommand() {

        return autoChooser.getSelected(); }

    /* 
    ╔══════════════════════════════════════════════════════════════════════════════════════════╗
    ║                            Create Drive-to-Pose Trigger                                  ║
    ╚══════════════════════════════════════════════════════════════════════════════════════════╝
    */
    private void createDriveToPoseTrigger(DoubleSupplier triggerSupplier, boolean isHPStation) {
    
        new Trigger(() -> triggerSupplier.getAsDouble() > 0.5)
            .whileTrue(new DriveToPoseCommand(
                    swerveSubsystem,
                    swerveConstants.MAX_SPEED,
                    m_driverController::getLeftY,
                    m_driverController::getLeftX,
                    m_driverController::getRightX,
                    m_driverController::getLeftTriggerAxis,
                    m_driverController::getRightTriggerAxis,
                    isHPStation,
                    false
            ));
    }

    /* 
    ╔══════════════════════════════════════════════════════════════════════════════════════════╗
    ║                           Create Drive-to-Pose Button Trigger                            ║
    ╚══════════════════════════════════════════════════════════════════════════════════════════╝
    */
    private void createDriveToPoseButtonTrigger(Trigger button, boolean HPStation) {
        button.whileTrue(new DriveToPoseCommand(
                swerveSubsystem,
                swerveConstants.MAX_SPEED,
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                m_driverController::getRightX,
                m_driverController::getLeftTriggerAxis,
                m_driverController::getRightTriggerAxis,
                HPStation,
                false
        ));
    }
    private void registerNamedCommands() {
        NamedCommands.registerCommand("setHomePose", new SequentialCommandGroup(
                new InstantCommand(sSlider::setRetract),
                new setElevatorPose(sElevator, elevatorConstants.kHomePose)
        )); 
        NamedCommands.registerCommand("placeL4", new SequentialCommandGroup(
                new setElevatorPose(sElevator, elevatorConstants.kL4Height).withTimeout(2.0),
                new InstantCommand(sSlider::setExtend),
                new WaitCommand(.5),
                new InstantCommand(sEndAffector::setPlace),
                new InstantCommand(sIntake::setCoralPlaceMode),
                new WaitCommand(.5),
                new InstantCommand(sIntake::setFeedIntake),
                new InstantCommand(sEndAffector::setZero),
                new InstantCommand(sSlider::setRetract),
                new WaitCommand(.5)
        ));

    }
}
