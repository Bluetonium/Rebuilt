package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class AutoAim {
    private CommandSwerveDrivetrain drivetrain;
    private RobotContainer robotContainer;

    private final SwerveRequest.FieldCentricFacingAngle driveAtAngle =
    new SwerveRequest.FieldCentricFacingAngle().withDeadband(RobotContainer.MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    private boolean simLineVisible;
    private boolean simArrowVisible;

    public AutoAim(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, RobotContainer robotContainer) {
        this.drivetrain = drivetrain;
        this.robotContainer = robotContainer;

        driveAtAngle.HeadingController.setPID(4, 0.0, 0.25);
        driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Rotation2d getAngleToHub() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d robotPos = robotPose.getTranslation();

        double dx = robotContainer.getHubX() - robotPos.getX();
        double dy = 4.034 - robotPos.getY();

        return new Translation2d(dx, dy).getAngle().plus(Rotation2d.fromDegrees(180));
    }

    public double getDistanceToHub() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d robotPos = robotPose.getTranslation();

        double dx = robotContainer.getHubX() - robotPos.getX();
        double dy = 4.034 - robotPos.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }

    public Command autoAimCommand() {
        return drivetrain.applyRequest(() -> {
            Rotation2d target = getAngleToHub();

            return driveAtAngle
                .withVelocityX(-RobotContainer.chassisController.getLeftY() * RobotContainer.MaxSpeed)
                .withVelocityY(-RobotContainer.chassisController.getLeftX() * RobotContainer.MaxSpeed)
                .withTargetDirection(target);
        });
    }

    public boolean isAimed() {
        Rotation2d target = getAngleToHub().plus(Rotation2d.fromDegrees(180));
        Rotation2d current = drivetrain.getState().Pose.getRotation();

        double error = MathUtil.angleModulus(
            target.getRadians() - current.getRadians()
        );

        return Math.abs(error) < Math.toRadians(10.0);
    }

    public Command align() {
        return Commands.parallel(
            autoAimCommand(),
            Commands.sequence(
                Commands.waitUntil(this::isAimed).withTimeout(3.0),
                RobotContainer.getShooter().runFlywheelAndLoader().withTimeout(7.0)
            )
        );
    }

    public void toggleSimAnnotations(){
        simLineVisible = !simLineVisible;
        simArrowVisible = !simArrowVisible;
    }
}
