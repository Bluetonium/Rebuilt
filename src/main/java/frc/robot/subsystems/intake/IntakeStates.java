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
        //RobotStates.toggleIntakePosition.onTrue(intake.toggleAngle(90, 00));

        RobotStates.intakeDown.onTrue(intake.moveDown());
        RobotStates.intakeUp.onTrue(intake.moveUp());


         
        //RobotStates.sysDyn.whileTrue(intake.intakeSysIdDynamic(Direction.kForward));
        //RobotStates.sysSta.whileTrue(intake.intakeSysIdQuasistatic(Direction.kForward));
        //RobotStates.sysDynRev.whileTrue(intake.intakeSysIdDynamic(Direction.kReverse));
        //RobotStates.sysStaRev.whileTrue(intake.intakeSysIdQuasistatic(Direction.kReverse));
        
    }

    

}
