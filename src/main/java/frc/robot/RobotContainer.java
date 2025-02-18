// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutonGrabCoral;
import frc.robot.commands.AutonPlaceCoral;
import frc.robot.commands.ClawDrop;
import frc.robot.commands.ClawIntake;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.GrabCoralHigh;
import frc.robot.commands.GrabCoralLow;
import frc.robot.commands.PlaceCoral;
import frc.robot.commands.SelectPlacement;
import frc.robot.commands.Store;
import frc.robot.commands.StorePreMatch;
import frc.robot.commands.ZeroAll;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroShoulder;
import frc.robot.commands.ZeroWrist;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    // Subsystems
    public final Shoulder m_shoulder = new Shoulder();
    public final Wrist m_wrist = new Wrist();
    public final Elevator m_elevator = new Elevator();
    public final Claw m_claw = new Claw();
    public final Vision m_Vision = new Vision();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // private CANdi wristAndClawCandi;
    // private CANdi shoulderAndTopCandi;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    // 3/4 of a rotation per second max angular velocity
    private double percentSlow = 1;

    public String goalArrangement = "blank";
    public String currentArrangement = "blank";

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0);
    public final XboxController accessory = new XboxController(1);
    // private final CommandXboxController characterizationJoystick = new
    // CommandXboxController(2);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* Fields */
    public Field2d field = new Field2d();
    public Field2d targetPoseField = new Field2d();

    // TODO reassign
    public int globalCurrNumSelected = 1;

    public RobotContainer() {

        NamedCommands.registerCommand("AutonPlaceCoral", new AutonPlaceCoral(m_shoulder, m_elevator, m_wrist, m_claw));
        NamedCommands.registerCommand("AutonGrabCoral", new AutonGrabCoral(m_shoulder, m_elevator, m_wrist, m_claw));

        // wristAndClawCandi = new CANdi(30, "rio");
        // shoulderAndTopCandi = new CANdi(31, "rio");

        autoChooser = AutoBuilder.buildAutoChooser("Autonomous Command");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // SmartDashboard Commands
        // SmartDashboard.putData("AutonGrabCoral", new InstantCommand(() ->
        // command).andThen(new Command()));
        // SmartDashboard.putData("AutonPlaceCoral", new InstantCommand(() ->
        // command).andThen(new Command()));
        SmartDashboard.putData("ClawDrop", new ClawDrop(m_claw));
        SmartDashboard.putData("ClawIntake", new ClawIntake(m_claw));
        SmartDashboard.putData("Climb", new InstantCommand(() -> goalArrangementOthers(PoseSetter.Climb))
                .andThen(new Climb(m_elevator)));
        SmartDashboard.putData("DriveToPosition", new DriveToPosition(drivetrain));
        SmartDashboard.putData("GrabCoralHigh", new InstantCommand(() -> goalArrangementOthers(PoseSetter.Feeder))
                .andThen(new GrabCoralHigh(m_shoulder, m_elevator, m_wrist, m_claw)));
        SmartDashboard.putData("GrabCoralLow", new InstantCommand(() -> goalArrangementOthers(PoseSetter.Ground))
                .andThen(new GrabCoralLow(m_shoulder, m_elevator, m_wrist, m_claw)));
        // SmartDashboard.putData("MoveElevator", new InstantCommand(() -> command)
        // .andThen(new MoveElevator(m_elevator)));
        // SmartDashboard.putData("MoveShoulder", new InstantCommand(() -> command)
        // .andThen(new MoveShoulder(m_shoulder)));
        // SmartDashboard.putData("MoveWrist", new InstantCommand(() -> command)
        // .andThen(new MoveWrist(m_wrist)));
        SmartDashboard.putData("PlaceCoral", new InstantCommand(() -> goalArrangementPlacing())
                .andThen(new PlaceCoral(m_shoulder, m_elevator, m_wrist, m_claw)));
        SmartDashboard.putData("Store", new InstantCommand(() -> goalArrangementOthers(PoseSetter.Stored))
                .andThen(new Store(m_shoulder, m_elevator, m_wrist, m_claw)));
        SmartDashboard.putData("StorePreMatch", new InstantCommand(() -> goalArrangementOthers(PoseSetter.Stored))
                .andThen(new StorePreMatch(m_shoulder, m_elevator, m_wrist, m_claw)));
        SmartDashboard.putData("ZeroAll", new ZeroAll(m_shoulder, m_elevator, m_wrist, m_claw));
        SmartDashboard.putData("ZeroElevator", new ZeroElevator(m_elevator));
        SmartDashboard.putData("ZeroShoulder", new ZeroShoulder(m_shoulder));
        SmartDashboard.putData("ZeroWrist", new ZeroWrist(m_wrist));

        // Field Widgets
        SmartDashboard.putData("Current Robot Position", field);
        SmartDashboard.putData("Target Robot Position", targetPoseField);

        // selector spots
        Constants.Selector.PlacementSelector.initializeTab();
        SmartDashboard.putString("current setting", currentArrangement);
        SmartDashboard.putString("goal Setting", goalArrangement);

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically

                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed * percentSlow)
                        // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed * percentSlow)
                        // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * percentSlow)
                // Drive counterclockwise with negative X (left)
                ));
        // generated buttons that drivers will probably never use
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));

        // Characterization buttons
        // Note that each routine should be run exactly once in a single log.
        // characterizationJoystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // characterizationJoystick.a().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // characterizationJoystick.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // characterizationJoystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Operator buttons
        joystick.leftTrigger(.5).onTrue(new InstantCommand(() -> goalArrangementPlacing())
                .andThen(new PlaceCoral(m_shoulder, m_elevator, m_wrist, m_claw)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        joystick.leftTrigger(.5).onFalse(new ClawDrop(m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        joystick.rightBumper().whileTrue(new InstantCommand(() -> goalArrangementOthers(PoseSetter.Feeder))
                .andThen(new GrabCoralHigh(m_shoulder, m_elevator, m_wrist, m_claw)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        joystick.rightTrigger(.5).whileTrue(new InstantCommand(() -> goalArrangementOthers(PoseSetter.Ground))
                .andThen(new GrabCoralLow(m_shoulder, m_elevator, m_wrist, m_claw)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        joystick.y().onTrue(new InstantCommand(() -> slow()));
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Op Test Buttons TODO Reassign
        joystick.b().whileTrue(
                new DriveToPosition(drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        joystick.leftBumper().onTrue(new InstantCommand(() -> minus()));
        joystick.a().onTrue(new InstantCommand(() -> plus()));

        // Accessory buttons
        final POVButton pOVButtonLeft = new POVButton(accessory, 270, 0);
        pOVButtonLeft.onTrue(new SelectPlacement(270).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        final POVButton pOVButtonRight = new POVButton(accessory, 90, 0);
        pOVButtonRight.onTrue(new SelectPlacement(90).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        final POVButton pOVButtonDown = new POVButton(accessory, 180, 0);
        pOVButtonDown.onTrue(new SelectPlacement(180).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        final POVButton pOVButtonUp = new POVButton(accessory, 0, 0);
        pOVButtonUp.onTrue(new SelectPlacement(0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        final JoystickButton btnClimb = new JoystickButton(accessory, XboxController.Button.kStart.value);
        btnClimb.onTrue(new InstantCommand(() -> goalArrangementOthers(PoseSetter.PreClimb))
                .andThen(new Climb(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        btnClimb.onFalse(new InstantCommand(() -> goalArrangementOthers(PoseSetter.Climb))
                .andThen(new Climb(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        final JoystickButton btnZeroAll = new JoystickButton(accessory, XboxController.Button.kBack.value);
        btnZeroAll.onTrue(new InstantCommand(() -> goalArrangementOthers(PoseSetter.Zero))
                .andThen(new ZeroAll(m_shoulder, m_elevator, m_wrist, m_claw)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        final JoystickButton btnStorePreMatch = new JoystickButton(accessory, XboxController.Button.kBack.value);
        btnStorePreMatch.onTrue(new InstantCommand(() -> goalArrangementOthers(PoseSetter.Stored))
                .andThen(new StorePreMatch(m_shoulder, m_elevator, m_wrist, m_claw)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        final JoystickButton btnStore = new JoystickButton(accessory, XboxController.Button.kA.value);
        btnStore.onTrue(new InstantCommand(() -> goalArrangementOthers(PoseSetter.Stored))
                .andThen(new Store(m_shoulder, m_elevator, m_wrist, m_claw)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void slow() {
        if (percentSlow == 1) {
            percentSlow = Constants.SwerveConstants.percentSlow;
        } else {
            percentSlow = 1;
        }
    }

    // TODO NAMES
    private void plus() {
        globalCurrNumSelected++;
    }

    private void minus() {
        if (globalCurrNumSelected > 1) {
            globalCurrNumSelected--;
        }
    }

    public String goalArrangementPlacing() {
        Robot.getInstance().m_elevator.elevatorStage1Target = PoseSetter.positionsMap
                .get(Constants.Selector.PlacementSelector.getLevel())[0];
        Robot.getInstance().m_elevator.elevatorStage2Target = PoseSetter.positionsMap
                .get(Constants.Selector.PlacementSelector.getLevel())[1];
        Robot.getInstance().m_shoulder.shoulderTarget = PoseSetter.positionsMap
                .get(Constants.Selector.PlacementSelector.getLevel())[2];
        Robot.getInstance().m_wrist.wristTarget = PoseSetter.positionsMap
                .get(Constants.Selector.PlacementSelector.getLevel())[3];
        goalArrangement = Constants.Selector.PlacementSelector.getLevel();
        SmartDashboard.putString("goal setting", goalArrangement);
        System.out.println("goal setting is " + goalArrangement);
        return goalArrangement;
    }

    public String goalArrangementOthers(String position) {
        Robot.getInstance().m_elevator.elevatorStage1Target = PoseSetter.positionsMap.get(position)[0];
        Robot.getInstance().m_elevator.elevatorStage2Target = PoseSetter.positionsMap.get(position)[1];
        Robot.getInstance().m_shoulder.shoulderTarget = PoseSetter.positionsMap.get(position)[2];
        Robot.getInstance().m_wrist.wristTarget = PoseSetter.positionsMap.get(position)[3];
        goalArrangement = position;
        SmartDashboard.putString("goal setting", goalArrangement);
        return goalArrangement;
    }

    public String currentArrangementPlacing() {
        currentArrangement = Constants.Selector.PlacementSelector.getLevel();
        SmartDashboard.putString("current setting", currentArrangement);
        return currentArrangement;
    }

    public String currentArrangementOthers(String position) {
        currentArrangement = goalArrangementOthers(position);
        SmartDashboard.putString("current setting", currentArrangement);
        return currentArrangement;
    }

    public Boolean getWristTripped() {
        return false;
        // return !shoulderAndTopCandi.getS2Closed().getValue();
    }

    public Boolean getTopStage2() {
        return false;
        // return shoulderAndTopCandi.getS1Closed().getValue();
    }

    public Boolean getCoralDetect() {
        return false;
        // return !wristAndClawCandi.getS1Closed().getValue();
    }
}
