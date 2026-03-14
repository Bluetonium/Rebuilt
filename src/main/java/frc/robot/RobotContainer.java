// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;

import lombok.Getter;

public class RobotContainer {
    private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double hubX;
                
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle driveAtAngle =
    new SwerveRequest.FieldCentricFacingAngle().withDeadband(MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandXboxController chassisController = new CommandXboxController(0);
    public final static CommandXboxController shootController = new CommandXboxController(1);
    //i will incorporate this later just dont want to forget or smth idrk
    public final static CommandXboxController pidController = new CommandXboxController(3);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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

    public void initHubPosition() {
        hubX = isRed() ? 11.9167 : 4.625;
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

        NamedCommands.registerCommand("align", align());
        NamedCommands.registerCommand("moveDown", intake.moveDown());
        NamedCommands.registerCommand("moveUp", intake.moveUp());
        NamedCommands.registerCommand("runForward", intake.runForward());
        NamedCommands.registerCommand("runBackward", intake.runBackward());

        autoChooser = AutoBuilder.buildAutoChooser();
        currentAuto = autoChooser.getSelected();
        autoChooser.onChange((command) -> currentAuto = command);
        SmartDashboard.putData("Autonomous", autoChooser);
    }

    private double aimIntegral = 0;
    private double lastTimestamp = Timer.getFPGATimestamp();

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

        driveAtAngle.HeadingController.setPID(7.5, 0.0, 0.25);
        driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

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
        RobotStates.autoAim.whileTrue(autoAimCommand());

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

    public Rotation2d getAngleToHub() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d robotPos = robotPose.getTranslation();

        double dx = hubX - robotPos.getX();
        double dy = 4.034 - robotPos.getY();

        return new Translation2d(dx, dy).getAngle();
    }


    public double getDistanceToHub() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d robotPos = robotPose.getTranslation();

        double dx = hubX - robotPos.getX();
        double dy = 4.034 - robotPos.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }

    public Command getAutonomousCommand() {
        return currentAuto;
    }

    private void initializeSubsystems() {
        shooter = new Shooter(this::getDistanceToHub);
        intake = new Intake();
    }

    private void setupSubsystems() {
        shooter.setup();
        intake.setup();
    }

    public Command toggleHubPosition() {
        return Commands.runOnce(() -> hubX = (hubX == 11.9167) ? 4.625 : 11.9167);
    }

    public Command autoAimCommand() {
        return drivetrain.applyRequest(() -> {

            Rotation2d target = getAngleToHub();
            Rotation2d current = drivetrain.getState().Pose.getRotation();

            double error = MathUtil.angleModulus(
                target.minus(current).getRadians()
            );

            double now = Timer.getFPGATimestamp();
            double dt = now - lastTimestamp;
            lastTimestamp = now;

            double kP = 8.0;
            double kI = 1.2;

            aimIntegral += error * dt;
            aimIntegral = MathUtil.clamp(aimIntegral, -0.4, 0.4);

            double turn = kP * error + kI * aimIntegral;

            double minTurn = 0.22;
            if (Math.abs(turn) < minTurn && Math.abs(error) > Math.toRadians(0.1)) {
                turn = Math.copySign(minTurn, error);
            }

            double maxTurnRate = 4.0;
            turn = MathUtil.clamp(turn, -maxTurnRate, maxTurnRate);

            if (Math.abs(error) < Math.toRadians(0.05)) {
                turn = 0;
                aimIntegral = 0;
            }

            return drive
                .withVelocityX(-chassisController.getLeftY() * MaxSpeed)
                .withVelocityY(-chassisController.getLeftX() * MaxSpeed)
                .withRotationalRate(turn);
        });
    }

    public Command align() {
        return Commands.sequence(
            autoAimCommand().withTimeout(5.0),
            shooter.runFlywheelAndLoader().withTimeout(14.0)
        );
    }
}
