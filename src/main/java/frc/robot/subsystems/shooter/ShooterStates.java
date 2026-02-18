package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;

public class ShooterStates {
    private static Shooter shooter = RobotContainer.getShooter();

    public static void setupStates() {
        RobotStates.runShooter.whileTrue(shooter.runForward());
        RobotStates.reverseShooter.whileTrue(shooter.runBackward());



        //SYS ID TRIGGER CALLS TO GET DATA FOR THE ANALLYZER <-------(Devin's spelling)
        /* 
        RobotStates.sysDyn.whileTrue(shooter.sysIdDynamic(Direction.kForward));
        RobotStates.sysSta.whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        RobotStates.sysDynRev.whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        RobotStates.sysStaRev.whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        */
        
    }

    

}
