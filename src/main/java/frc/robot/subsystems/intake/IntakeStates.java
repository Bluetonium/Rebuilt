package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;

@SuppressWarnings("unused")
public class IntakeStates {
    private static Intake intake = RobotContainer.getIntake();

    public static void setupStates() {

        RobotStates.runIntake.whileTrue(intake.runForward());
        RobotStates.reverseIntake.whileTrue(intake.runBackward());

        /* 
        RobotStates.sysDyn.whileTrue(intake.sysIdDynamic(Direction.kForward));
        RobotStates.sysSta.whileTrue(intake.sysIdQuasistatic(Direction.kForward));
        RobotStates.sysDynRev.whileTrue(intake.sysIdDynamic(Direction.kReverse));
        RobotStates.sysStaRev.whileTrue(intake.sysIdQuasistatic(Direction.kReverse));
        */
    }

    

}
