// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;

import lombok.Getter;

public class RobotContainer {
    public static double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
                
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandXboxController chassisController = new CommandXboxController(0);
    public final static CommandXboxController shootController = new CommandXboxController(1);
    //i will incorporate this later just dont want to forget or smth idrk
    public final static CommandXboxController pidController = new CommandXboxController(3);

    private double hubX = initHubPosition();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final AutoAim autoaim = new AutoAim(drivetrain, hubX, drive);

    private final SendableChooser<Command> autoChooser;
    @Getter
    private static Command currentAuto;

    public static boolean isRed() {
        return DriverStation.isFMSAttached()
            ? DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            : NetworkTableInstance.getDefault()
                .getTable("FMSInfo")
                .getEntry("IsRedAlliance")
                .getBoolean(false);
    }

    public double initHubPosition() {
        return isRed() ? 11.9167 : 4.625;
    }

    public double getHubX() {
        return hubX;
    }

    @Getter
    private static Shooter shooter;

    @Getter
    private static Intake intake;

    public RobotContainer() {
        initializeSubsystems();
        RobotStates.setupStates();
        configureBindings();
        setupSubsystems();

        NamedCommands.registerCommand("align", autoaim.align());
        NamedCommands.registerCommand("moveDown", intake.moveDown());
        NamedCommands.registerCommand("moveUp", intake.moveUp());
        NamedCommands.registerCommand("runForward", intake.runForward());
        NamedCommands.registerCommand("runBackward", intake.runBackward());

        autoChooser = AutoBuilder.buildAutoChooser();
        currentAuto = autoChooser.getSelected();
        autoChooser.onChange((command) -> currentAuto = command);
        SmartDashboard.putData("Autonomous", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-chassisController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-chassisController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-chassisController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Brake
        RobotStates.brake.whileTrue(drivetrain.applyRequest(() -> brake));
        // chassisController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-chassisController.getLeftY(), -chassisController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*chassisController.back().and(chassisController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        chassisController.back().and(chassisController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        chassisController.start().and(chassisController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        chassisController.start().and(chassisController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        // Reset the field-centric heading on right bumper press.
        RobotStates.resetHeading.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        RobotStates.slowMode.whileTrue(
            Commands.startEnd(
                () -> MaxSpeed = 0.2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                () -> MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                // ← no subsystem passed here, so no requirements
            )
        );

        // Autoaim
        RobotStates.autoAim.whileTrue(autoaim.autoAimCommand());

        RobotStates.toggleHubPosition.onTrue(toggleHubPosition());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // im tired of having only dry water and wet rice
    // public double getAngleToHub() {
    //     Pose2d robotPose = drivetrain.getState().Pose;
    //     Translation2d robotPos = robotPose.getTranslation();

    //     double dx = hubX - robotPos.getX();
    //     double dy = 4.034 - robotPos.getY();

    //     /*boolean shouldInvert = !isRed && DriverStation.isTeleopEnabled();
    //     double angle = ((Math.toDegrees(Math.atan2(dy, dx)) + 180 + 180) % 360);

    //     //return ((Math.toDegrees(Math.atan2(dy, dx)) + 180 + 180) % 360) - 180;
    //     return shouldInvert ? angle - 180 : angle;*/
        
    //     double angle = ((Math.toDegrees(Math.atan2(dy, dx)) + 180 + 180) % 360) - 180;
    //     return !isRed() && DriverStation.isTeleopEnabled() ? angle + 180 : angle;
    // }

    public Command getAutonomousCommand() {
        return currentAuto;
    }

    private void initializeSubsystems() {
        shooter = new Shooter(autoaim::getDistanceToHub);
        intake = new Intake();
    }

    private void setupSubsystems() {
        shooter.setup();
        intake.setup();
    }

    public Command toggleHubPosition() {
        return Commands.runOnce(() -> hubX = (hubX == 11.9167) ? 4.625 : 11.9167);
    }
}
