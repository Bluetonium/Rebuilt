package frc.robot;

import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Setter;

public class RobotStates {
    public static class DriverConfigs {
        @Setter
        protected static int port = 0;
        @Setter
        protected static boolean elevatorControl = false;
        @Setter
        protected static boolean chassisDriving = false;
        @Setter
        protected static boolean outtakeControl = false;
    }

    //LAST DITCH CASE SCENARIO
    //private static final XboxController controller = new XboxController(DriverConfigs.port);

    public RobotStates() {
        setupStates();
    }

    // states
    public static Trigger teleop;
    public static Trigger autoMode;
    public static Trigger testMode;
    public static Trigger disabled;
    public static Trigger dsAttached;
    public static Trigger endGame;
    public static Trigger Estopped;
    

    // shooter
    public static Trigger runShooter;
    public static Trigger reverseShooter;

    public static Trigger loadShooter;
    public static Trigger unloadShooter;

    public static Trigger runIntake;
    public static Trigger reverseIntake;

    public static Trigger sysDyn;
    public static Trigger sysSta;
    public static Trigger sysDynRev;
    public static Trigger sysStaRev;

    private static BooleanSupplier isRightTriggerDown2() {        
        return () -> RobotContainer.shootController.getRightTriggerAxis() > 0.1;
    }

    private static BooleanSupplier isLeftTriggerDown2() {        
        return () -> RobotContainer.shootController.getLeftTriggerAxis() > 0.1;
    }


    public static void setupStates() {
        teleop = new Trigger(DriverStation::isTeleopEnabled);
        autoMode = new Trigger(RobotState::isAutonomous);
        testMode = new Trigger(RobotState::isTest);
        disabled = new Trigger(RobotState::isDisabled);
        dsAttached = new Trigger(DriverStation::isDSAttached);
        Estopped = new Trigger(DriverStation::isEStopped);

        endGame = teleop.and(() -> DriverStation.getMatchTime() < 20);

        runShooter = new Trigger(isRightTriggerDown2());
        reverseShooter = new Trigger(isLeftTriggerDown2());

        
        runIntake = RobotContainer.shootController.rightBumper();
        reverseIntake = RobotContainer.shootController.leftBumper();


        sysDyn = new Trigger(RobotContainer.shootController.x());
        sysSta = new Trigger(RobotContainer.shootController.y());
        sysDynRev = new Trigger(RobotContainer.shootController.a());
        sysStaRev = new Trigger(RobotContainer.shootController.b());
    }
}
