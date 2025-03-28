package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.fieldPoses;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.robotConstants.elevatorConstants;
import frc.robot.commands.SwerveCommands.aprilTagSwerve;
import frc.robot.commands.VariableAutos.BranchSide;
import frc.robot.commands.setCHaptics;
import frc.robot.commands.setCHapticsAndHold;
import frc.robot.commands.climberCommands.setClimber;
import frc.robot.commands.elevatorCommands.setElevatorOffset;
import frc.robot.commands.elevatorCommands.setElevatorPose;
import frc.robot.commands.intakeCommands.setIntake;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.commands.SwerveCommands.driveToPose;
import frc.robot.commands.autos.AlignToReef;
import frc.robot.commands.autos.AlignToReef.FieldBranchSide;
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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
╔══════════════════════════════════════════════════════════════════════════════════════╗
║               RobotContainer Class  ::  This is like our main class                  ║
║                              creates button binds and commands                       ║ 
╚══════════════════════════════════════════════════════════════════════════════════════╝
*/
public class RobotContainer {
    sEndAffector sEndAffector;
    sSlider sSlider;
    sClimber sClimber;
    sElevator sElevator;
    SwerveSubsystem swerveSubsystem;
    sIntake sIntake;
    int cameraID;
    setIntake defaultIntake, intakeBall;
    setClimber defaultClimber;
    sControllerHaptics m_controllerHaptics;
    setElevatorPose setL4Pose, setL3Pose, setL2Pose, setL1Pose, setHomePose, setProcessorPose, setCoralPose;
    DriveToPoseCommand autoDriveToPoseCommand;
    setElevatorOffset setElevatorOffset;
    Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
    private AlignToReef alignmentCommandFactory;
     public static final AprilTagFieldLayout fieldLayout                     = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeWelded);  

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
        //sEndAffector = new sEndAffector();
        sClimber = new sClimber();
        sElevator = new sElevator();
        m_controllerHaptics = new sControllerHaptics(m_driverController, m_operatorController);
        sIntake = new sIntake();

        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                "neo"));
        setupCommands();
        configureBindings();
        registerNamedCommands();

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
        alignmentCommandFactory = new AlignToReef(swerveSubsystem, fieldLayout);

        //intakeBall = new setIntake(false, true, 0.0, -1.0, sEndAffector);

        // Elevator Commands
        setL4Pose = new setElevatorPose(sElevator, elevatorConstants.kL4Height, ()-> m_operatorController.getLeftY());
        setL3Pose = new setElevatorPose(sElevator, elevatorConstants.kL3Height, ()-> m_operatorController.getLeftY());
        setL2Pose = new setElevatorPose(sElevator, elevatorConstants.kL2Height, ()-> m_operatorController.getLeftY());
        setL1Pose = new setElevatorPose(sElevator, elevatorConstants.kL1Height, ()-> m_operatorController.getLeftY());
        setHomePose = new setElevatorPose(sElevator, elevatorConstants.kHomePose, ()-> m_operatorController.getLeftY());
        setProcessorPose = new setElevatorPose(sElevator, elevatorConstants.kProcessorhHeight, ()-> m_operatorController.getLeftY());
        setCoralPose = new setElevatorPose(sElevator, elevatorConstants.kCoralFeedPose, ()-> m_operatorController.getLeftY());

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
        
        driveControls();
        newVisionControls();
        //DISABLE VISION
        m_driverController.start().and(m_driverController.back()).onTrue(new InstantCommand(swerveSubsystem::stopVision));
        //`````````````````````````````````````````````````
        //`````````Driver shooting controls````````````````
        //`````````````````````````````````````````````````
        m_driverController.rightBumper().onTrue(
            new SequentialCommandGroup(
             new InstantCommand(sSlider::setExtend))) 
            .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(sIntake::setFeedIntake),
            new WaitCommand(.5),
            new setCHapticsAndHold(m_controllerHaptics,.8),
                new InstantCommand(sIntake::setZero),
                new InstantCommand(sSlider::setRetract), 
                new InstantCommand(sIntake::hardResetIntake),
            new WaitCommand(.5),
            new setCHapticsAndHold(m_controllerHaptics,0)
            //TODO: discuss this with the team
            ).withTimeout(4.0));  

           
        createDriveToPoseButtonTrigger(m_driverController.x(), true);
        // m_driverController.b().whileTrue(new driveToPose(swerveSubsystem,fieldPoses.reefPose));
    }
    public void driveControls(){
        // Set default drive command

            // Flip controls if on Red Alliance

            swerveSubsystem.setDefaultCommand(
            swerveSubsystem.driveCommand(
            () ->  MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1),
            () ->  MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1),
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1)
       
            )
            );
            m_driverController.back().onTrue(new InstantCommand(swerveSubsystem::flipDrive));

            m_driverController.leftBumper()
            .whileTrue(
                 swerveSubsystem.driveRobotCentCommand(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1)*.10,
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1)*.10,
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1)*.350)
            );

            //TODO: FIX THIS
            // m_driverController.leftTrigger(.2).whileTrue(new aprilTagSwerve(swerveSubsystem,
            //     ()->-m_driverController.getLeftX(),()->-m_driverController.getLeftY(),()->m_driverController.getRightX(),
            //     ()->false,m_driverController,m_operatorController,fieldPoses.lSidePose, robotConstants.ilCameraID));

            // m_driverController.rightTrigger(.2).whileTrue(new aprilTagSwerve(swerveSubsystem,
            //                 ()->-m_driverController.getLeftX(),()->-m_driverController.getLeftY(),()->m_driverController.getRightX(),
            //                 ()->false,m_driverController,m_operatorController,fieldPoses.rSidePose,robotConstants.irCameraID));

            //sElevator.setDefaultCommand(sElevator.setElevator(() -> MathUtil.applyDeadband(m_operatorController.getLeftY(), .1)));
            m_driverController.a().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
            m_driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyroWithAlliance));
    }
    public void newVisionControls(){
        m_driverController.leftTrigger(.2).whileTrue(
            alignmentCommandFactory.generateCommand(FieldBranchSide.LEFT).andThen(
                swerveSubsystem.driveRobotCentCommand(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1)*.10+.05,
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1)*.10,
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1)*.350
            )
            )//.finallyDo((boolean interrupted) -> {
            //     dynamics.gotoLastInputtedScore().onlyIf(() -> !interrupted);
            // })
            .withName("Align Left Branch")
        );

        m_driverController.rightTrigger(.2).whileTrue(
            alignmentCommandFactory.generateCommand(FieldBranchSide.RIGHT)
            .andThen(swerveSubsystem.driveRobotCentCommand(
                () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1)*.10+.05,
                () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1)*.10,
                () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1)*.350
                ))//.finallyDo((boolean interrupted) -> {
            //     dynamics.gotoLastInputtedScore().onlyIf(() -> !interrupted);
            // })
            .withName("Align Right Branch")
        );
    }

    /* 
    ╔════════════════════════════════════════════════════════════════════════════════════════╗
    ║                                 Operator Control Configuration                         ║
    ╚════════════════════════════════════════════════════════════════════════════════════════╝
    */
    public void operatorControls() {
        
        // ------------------------- Preset Pose Commands ------------------------- //
        // m_operatorController.a().onTrue(setL1Pose);
        m_operatorController.y().onTrue(setL4Pose.withTimeout(3.0));          
        m_operatorController.b().onTrue(setL3Pose.withTimeout(3.0));                                               
        m_operatorController.x().onTrue(setL2Pose.withTimeout(3.0));
        m_operatorController.a().onTrue(setCoralPose.withTimeout(3.0));
        m_operatorController.start().onTrue(new SequentialCommandGroup(
            new setElevatorPose(sElevator, 1.0,()->0),
            setHomePose.withTimeout(3.0)));
        
        // ----------------------- Climber Commands ----------------------- //
        new Trigger(() -> m_operatorController.rightStick().getAsBoolean() &&
                          m_operatorController.leftStick().getAsBoolean())
                                .onTrue(new SequentialCommandGroup(new ParallelCommandGroup(
                                        new setCHaptics(m_controllerHaptics, 0.8).withTimeout(.75),
                                        new InstantCommand(sClimber::disableSafety),
                                        new InstantCommand(sClimber::dropRamp)),
                                        new InstantCommand(sClimber::deployClimber))                        
                                        );

        m_operatorController.povDown().onTrue(new ParallelCommandGroup(new InstantCommand(sClimber::unClimb),
                                                                       new InstantCommand(sClimber::stowClimber)))
                                                                       .onFalse(new InstantCommand(sClimber::zeroClimb));
        m_operatorController.povUp().onTrue(new ParallelCommandGroup(new InstantCommand(sClimber::climb),
                                                                     new InstantCommand(sClimber::deployClimber)))
                                                                     .onFalse(new InstantCommand(sClimber::zeroClimb));
        m_operatorController.povLeft().onTrue(new InstantCommand(sClimber::stowClimber));
        m_operatorController.povRight().onTrue(new InstantCommand(sClimber::deployClimber));
        // m_operatorController.start().onTrue(new InstantCommand(sClimber::dropRamp));
    
        // --------------------------- Intake Controls --------------------------- //
        m_operatorController.rightStick().onTrue(new InstantCommand(sIntake::hardResetIntake));

        m_operatorController.leftBumper()
                .onTrue(new ParallelCommandGroup(
                    new InstantCommand(sIntake::setBallIntake),
                    new setCHapticsAndHold(m_controllerHaptics, 0.2),
                    new InstantCommand(sSlider::setExtend)))
                .onFalse(new ParallelCommandGroup(new InstantCommand(sSlider::setRetract),
                                                  new InstantCommand(sIntake::setZero),
                                                  new setCHapticsAndHold(m_controllerHaptics,.0))); // No haptics here since motors are off
        m_operatorController.leftStick().onTrue(opeartorOffset);

        // **Left Trigger (≥ 0.2)**: Deploy & run **intake forward, place motor in reverse**
        m_operatorController.leftTrigger(0.2)
                .whileTrue(new ParallelCommandGroup(
                        new InstantCommand(sIntake::setManReverse),
                        new setCHaptics(m_controllerHaptics, 0.2))
                        )
                        .onFalse(new InstantCommand(sIntake::setZero)); // Haptic feedback when motors are on
        m_operatorController.rightTrigger(0.2)
                        .whileTrue(new ParallelCommandGroup(
                                new InstantCommand(sIntake::setManFeed),
                                new setCHaptics(m_controllerHaptics, 0.2))
                                )
                                .onFalse(new InstantCommand(sIntake::setZero)); // Haptic feedback when motors are on
        // m_operatorController.rightTrigger(0.2)
        //                 .whileTrue(new ParallelCommandGroup(
        //                         new InstantCommand(sEndAffector::setPlace),
        //                         new setCHaptics(m_controllerHaptics, 0.2))
        //                         ).onFalse(new ParallelCommandGroup(new InstantCommand(sEndAffector::setZero),
        //                                                                 new InstantCommand(sIntake::hardResetIntake)
        //                         )); // Haptic feedback when motors are on
        RobotModeTriggers.teleop().onTrue(new InstantCommand(sIntake::hardResetIntake));
        //RobotModeTriggers.teleop().onTrue(new setElevatorPose(sElevator,elevatorConstants.kProcessorhHeight));//TODO: REIMPLEMENT\\ maybe this crashes code though
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
        )).onFalse(new InstantCommand(swerveSubsystem::stop).withTimeout(0));
    }
    private void registerNamedCommands() {
        NamedCommands.registerCommand("initAuto", new SequentialCommandGroup(
            new setElevatorPose(sElevator, 1,()->0).withTimeout(.1),
            new setElevatorPose(sElevator,elevatorConstants.kHomePose,()->0).withTimeout(.5)
        ));
        NamedCommands.registerCommand("setHomePose", new SequentialCommandGroup(
                new InstantCommand(sSlider::setRetract),
                new setElevatorPose(sElevator, elevatorConstants.kCoralFeedPose,()->0).withTimeout(.5),
                new InstantCommand(sIntake::hardResetIntake)
        ).withTimeout(2.5)); 
        NamedCommands.registerCommand("placeL4", 
        new ParallelDeadlineGroup(new SequentialCommandGroup(

                //setting elevator to L4 height and extending the slider
                new setElevatorPose(sElevator, elevatorConstants.kL4Height,()->0).withTimeout(1.0),
                new InstantCommand(sSlider::setExtend),
                new WaitCommand(1.25),
                //actually feeding the intake
                new InstantCommand(sIntake::setFeedIntake),
                new WaitCommand(.125),
                new setElevatorOffset(sElevator, ()->-0.5).withTimeout(0.0),
                new WaitCommand(.75),
                //going back down
                new InstantCommand(sSlider::setRetract),
                new WaitCommand(.5)
        ), swerveSubsystem.driveRobotCentCommand(()->.2, ()->0,()->0).withTimeout(1.5)));
    }
    public static AprilTagFieldLayout getFieldlayout(){
        return fieldLayout;
    }

}
            //backup//
        // m_driverController.rightBumper().onTrue(new ParallelCommandGroup(new InstantCommand(sIntake::setFeedIntake)
                                                                         
        //                                                                 ))
        //                                   .onFalse(new ParallelCommandGroup(new InstantCommand(sIntake::setZero)
                                         
        //                                   ).withTimeout(0));  

        // Create DriveToPoseCommand based on trigger inputs
        // createDriveToPoseTrigger(m_driverController::getRightTriggerAxis, false);
        // createDriveToPoseTrigger(m_driverController::getLeftTriggerAxis, false);
        
        // Bind B button to drive to nearest AprilTag pose at center
        //createDriveToPoseButtonTrigger(m_driverController.b(), false);

        // Bind right bumper to drive to nearest AprilTag pose with a special mode
        //createDriveToPoseButtonTrigger(m_driverController.rightBumper(), true);
