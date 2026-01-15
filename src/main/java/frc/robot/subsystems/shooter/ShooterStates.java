package frc.robot.subsystems.shooter;

import frc.robot.RobotContainer;
import frc.robot.RobotStates;

public class ShooterStates {
    private static Shooter shooter = RobotContainer.getShooter();

    public static void setupStates() {
        RobotStates.runShooter.whileTrue(shooter.runForward());
        RobotStates.reverseShooter.whileTrue(shooter.runBackward());
    }

}
