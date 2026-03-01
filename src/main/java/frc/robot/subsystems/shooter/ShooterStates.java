package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;

@SuppressWarnings("unused")
public class ShooterStates {
    private static Shooter shooter = RobotContainer.getShooter();

    public static void setupStates() {
        RobotStates.unjamShooter.whileTrue(shooter.unjamShooter());

        RobotStates.primeShooter.and(RobotStates.fireShooter.negate()).whileTrue(shooter.runFlywheel());
        RobotStates.fireShooter.whileTrue(shooter.runFlywheelAndLoader());


        //Triggers to get PID values and whatnot
        /* 
        RobotStates.sysDyn.whileTrue(shooter.sysIdDynamic(Direction.kForward));
        RobotStates.sysSta.whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        RobotStates.sysDynRev.whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        RobotStates.sysStaRev.whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        */
        
    }

    

}
