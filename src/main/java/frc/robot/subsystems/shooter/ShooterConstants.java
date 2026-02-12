package frc.robot.subsystems.shooter;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.utils.sim.RollerConfig;

public class ShooterConstants {

    /**
     *TODO  Might be useful to put a comment here labeling what units these are
     * i kinda forget
     */
    public static final double ACCELERATION = 90;

    // flywheel constants
    public static final double FLYWHEEL_FORWARD_VELOCITY = -10;
    public static final double FLYWHEEL_BACKWARD_VELOCITY = 10;

    public static final int FLYWHEEL_MOTOR_CAN_ID = 16;
    public static final NeutralModeValue FLYWHEEL_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

    // loader constants
    public static final double LOADER_FORWARD_VELOCITY = -10;
    public static final double LOADER_BACKWARD_VELOCITY = 10;

    public static final int LOADER_MOTOR_CAN_ID = 18;
    public static final NeutralModeValue LOADER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;


    public static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)// holy hell man this thing doesnt need 1,400 watts of power,
            // this should be like maybe 50-60, do some testing, more current = more power,
            // its current got the same power as like a drivetrain, wayy overkill
            // Its also a safety thing, it should only be as strong as it needs to be
            .withSupplyCurrentLimit(30);// keep this bellow 40 idealy, this is how much power it should be able to take
                                        // from the PDH, 60 is too much (ps all of the supply currents of everything
                                        // should idealy be <120)

    public static final double GEAR_RATIO = 1;

    // PID
    public static final double kP = 0.038024;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.079022;
    public static final double kV = 0.12114;
    public static final double kA = 0.0028202;

    public static final RollerConfig ROLLER_SIM_CONFIG = new RollerConfig(4)
            .setPosition(1.341, .35);
}
