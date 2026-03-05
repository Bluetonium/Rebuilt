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


        
        
        RobotStates.sysDyn.whileTrue(shooter.transferSysIdDynamic(Direction.kForward));
        RobotStates.sysSta.whileTrue(shooter.transferSysIdQuasistatic(Direction.kForward));
        RobotStates.sysDynRev.whileTrue(shooter.transferSysIdDynamic(Direction.kReverse));
        RobotStates.sysStaRev.whileTrue(shooter.transferSysIdQuasistatic(Direction.kReverse));
        
        
    }

    

}
