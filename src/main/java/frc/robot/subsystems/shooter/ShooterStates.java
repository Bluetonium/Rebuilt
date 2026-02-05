package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;

public class ShooterStates {
    private static Shooter shooter = RobotContainer.getShooter();

    public static void setupStates() {
        RobotStates.runShooter.whileTrue(shooter.runForward());
        RobotStates.reverseShooter.whileTrue(shooter.runBackward());

        // TODO fixing naming to not be aberivated, (sysDynamic and sysQuasistatic),
        // also create a forward and backwards routine
        // or maybe you could do something like this or just bind to more buttons,
        // completely up to yall
        /*
         * RobotStates.sysDyn.whileTrue(shooter.sysIdDynamic(Direction.kForward).
         * andThen(
         * Commands.waitSeconds(5).andThen(//a wait in here to allow the motor to stop
         * spinning and return to 0 velocity
         * shooter.sysIdDynamic(Direction.kReverse))));
         * 
         */

        RobotStates.sysDyn.whileTrue(shooter.sysIdDynamic(Direction.kForward));
        RobotStates.sysSta.whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
    }

}
