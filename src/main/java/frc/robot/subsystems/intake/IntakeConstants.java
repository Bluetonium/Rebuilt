package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.utils.sim.RollerConfig;

public class IntakeConstants {

        //this motor is
    public static final double INTAKE_ACCELERATION = 90;
    public static final double INTAKE_FORWARD_VELOCITY = -10;
    public static final double INTAKE_BACKWARD_VELOCITY = 10;

    public static final int INTAKE_MOTOR_CAN_ID = 17;
    public static final NeutralModeValue INTAKE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final CurrentLimitsConfigs INTAKE_CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)
            .withSupplyCurrentLimit(30);

    public static final double INTAKE_GEAR_RATIO = 1;

    // PID
    public static final double kP_intake = 0.0034178;
    public static final double kI_intake = 0;
    public static final double kD_intake = 0;

    public static final double kS_intake = 0.044684;
    public static final double kV_intake = 0.093353;
    public static final double kA_intake = 0.0011177;

    public static final RollerConfig INTAKE_ROLLER_SIM_CONFIG = new RollerConfig(4)
            .setPosition(2, .7);

    //-----------------------------
    //intake dropper
    //-----------------------------
    public static final double INTAKE_DROPPER_ACCELERATION = 90;
    public static final double INTAKE_DROPPER_FORWARD_VELOCITY = -10;
    public static final double INTAKE_DROPPER_BACKWARD_VELOCITY = 10;

    public static final int INTAKE_DROPPER_MOTOR_CAN_ID = 18;
    public static final NeutralModeValue INTAKE_DROPPER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final CurrentLimitsConfigs INTAKE_DROPPER_CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)
            .withSupplyCurrentLimit(30);

    public static final double INTAKE_DROPPER_GEAR_RATIO = 1;

    //TODO TUNE THESE VALUES PEON
    public static final double kP_dropper = 0.0034178;
    public static final double kI_dropper = 0;
    public static final double kD_dropper = 0;

    public static final double kS_dropper = 0.044684;
    public static final double kV_dropper = 0.093353;
    public static final double kA_dropper = 0.0011177;

    public static final RollerConfig INTAKE_DROPPER_ROLLER_SIM_CONFIG = new RollerConfig(4)
            .setPosition(1, 1.7);
}
