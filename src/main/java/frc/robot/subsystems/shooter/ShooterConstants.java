package frc.robot.subsystems.shooter;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.utils.sim.RollerConfig;

public class ShooterConstants {

    /**
     *TODO  Might be useful to put a comment here labeling what units these are
     * i kinda forget
     */
    

    // flywheel constants
    public static final double FLYWHEEL_ACCELERATION = 90;
    public static final double FLYWHEEL_FORWARD_VELOCITY = -90;
    public static final double FLYWHEEL_BACKWARD_VELOCITY = 90;

    public static final int FLYWHEEL_MOTOR_CAN_ID = 16;
    public static final NeutralModeValue FLYWHEEL_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

    // loader constants
    public static final double LOADER_ACCELERATION = 90;
    public static final double LOADER_FORWARD_VELOCITY = -10;
    public static final double LOADER_BACKWARD_VELOCITY = 10;

    public static final int LOADER_MOTOR_CAN_ID = 18;
    public static final NeutralModeValue LOADER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;



    //current limits (should be same for both motors)
    public static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)// this should be like maybe 50-60, do some testing, more current = more power,
            .withSupplyCurrentLimit(30);// keep this bellow 40 ideally

    public static final double GEAR_RATIO = 1;

    // PID for Flywheel
    public static final double kP = 0.030071;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.17242;
    public static final double kV = 0.12144;
    public static final double kA = 0.0032487;

    // PID for Loader
    public static final double kP_loader = 0.038024;
    public static final double kI_loader = 0;
    public static final double kD_loader = 0;

    public static final double kS_loader = 0.079022;
    public static final double kV_loader = 0.12114;
    public static final double kA_loader = 0.0028202;



    public static final RollerConfig FLYWHEEL_ROLLER_SIM_CONFIG = new RollerConfig(4)
            .setPosition(1.341, .35);

    public static final RollerConfig LOADER_ROLLER_SIM_CONFIG = new RollerConfig(4)
            .setPosition(1.35,0.15);
}
