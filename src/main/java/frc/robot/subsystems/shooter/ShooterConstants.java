package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.utils.sim.RollerConfig;

public class ShooterConstants {
    public static final double ACCELERATION = 90;
    public static final double FORWARD_VELOCITY = -5;
    public static final double BACKWARD_VELOCITY =4;

    public static final int SHOOTER_MOTOR_CAN_ID = 16;
    public static final NeutralModeValue SHOOTER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(120)
        .withSupplyCurrentLimit(60);

    public static final double GEAR_RATIO = 1;

    // PID
    public static final double kP = 0.69993;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.17299;
    public static final double kV = 0.47682;
    public static final double kA = 0.027378;

    public static final RollerConfig ROLLER_SIM_CONFIG = new RollerConfig(4)
                    .setPosition(1.341, .35);
}
