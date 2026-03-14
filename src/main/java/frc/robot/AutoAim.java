package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class AutoAim {
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentric drive;
    private RobotContainer robotContainer;

    private double aimIntegral = 0;
    private double lastTimestamp = Timer.getFPGATimestamp();

    private boolean simLineVisible;
    private boolean simArrowVisible;

    public AutoAim(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, RobotContainer robotContainer) {
        this.drivetrain = drivetrain;
        this.drive = drive;
        this.robotContainer = robotContainer;
    }

    public Rotation2d getAngleToHub() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d robotPos = robotPose.getTranslation();

        double dx = robotContainer.getHubX() - robotPos.getX();
        double dy = 4.034 - robotPos.getY();

        return new Translation2d(dx, dy).getAngle();
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
                .withVelocityX(-RobotContainer.chassisController.getLeftY() * RobotContainer.MaxSpeed)
                .withVelocityY(-RobotContainer.chassisController.getLeftX() * RobotContainer.MaxSpeed)
                .withRotationalRate(turn);
        });
    }

    public Command align() {
        return Commands.sequence(
            autoAimCommand().withTimeout(5.0),
            RobotContainer.getShooter().runFlywheelAndLoader().withTimeout(14.0)
        );
    }

    public void toggleSimAnnotations(){
        simLineVisible = !simLineVisible;
        simArrowVisible = !simArrowVisible;
    }
}
