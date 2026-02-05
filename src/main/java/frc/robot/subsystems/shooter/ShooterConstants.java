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
    public static final double FORWARD_VELOCITY = -1;
    public static final double BACKWARD_VELOCITY = 1;

    public static final int SHOOTER_MOTOR_CAN_ID = 16;
    public static final NeutralModeValue SHOOTER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

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
    //bro just stole last years pid values lmao
    public static final double kP = 0.69993;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.17299;
    public static final double kV = 0.47682;
    public static final double kA = 0.027378;

    public static final RollerConfig ROLLER_SIM_CONFIG = new RollerConfig(4)
            .setPosition(1.341, .35);
}
