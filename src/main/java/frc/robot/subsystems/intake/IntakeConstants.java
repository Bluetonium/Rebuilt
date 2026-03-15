package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.utils.sim.ArmConfig;
import frc.utils.sim.ArmSim;
import frc.utils.sim.RollerConfig;
import frc.utils.sim.RollerSim;

public class IntakeConstants {

    //-----------------------------
    // Intake dropper arm (X60)
    //-----------------------------

    public static final double INTAKE_DOWN_ANGLE = 0.0;
    public static final double INTAKE_FUNNEL_ANGLE = 40.0;
    public static final double INTAKE_UP_ANGLE = 90.0;

    public static final double INTAKE_DROPPER_ACCELERATION = 1.5;
    public static final double INTAKE_DROPPER_CRUISE_VELOCITY = 1; // rotations/sec of mechanism
    public static final double INTAKE_DROPPER_GEAR_RATIO = 81;
    public static final double INTAKE_DROPPER_INITIAL_ANGLE = 90.0;

    public static final int INTAKE_DROPPER_MOTOR_CAN_ID = 20;
    public static final NeutralModeValue INTAKE_DROPPER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final CurrentLimitsConfigs INTAKE_DROPPER_CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)
            .withSupplyCurrentLimit(30);

    public static final double DROPPER_ARM_LENGTH_METERS = 0.3;     // measure your arm

    public static ArmSim intakeDropperArmSim;
    public static final ArmConfig INTAKE_DROPPER_SIM_CONFIG = new ArmConfig(
        1.6,   // pivotX on the mechanism2d
        0.5,   // pivotY
        IntakeConstants.INTAKE_DROPPER_GEAR_RATIO,
        IntakeConstants.DROPPER_ARM_LENGTH_METERS,
        0,     // min angle degrees
        90,    // max angle degrees
        INTAKE_DROPPER_INITIAL_ANGLE      // starting angle degrees
    );

    public static final double kP_dropper = 40.0;
    public static final double kI_dropper = 0;
    public static final double kD_dropper = 0.5;
    public static final double kS_dropper = 0.25;
    public static final double kV_dropper = 0.12;
    public static final double kA_dropper = 0.01;
    
    //-----------------------------
    // Intake roller (X44)
    //-----------------------------

    public static final double INTAKE_ACCELERATION = 60;
    public static final double INTAKE_FORWARD_VELOCITY = 70;
    public static final double INTAKE_BACKWARD_VELOCITY = -70;
    public static final double INTAKE_GEAR_RATIO = 1;

    public static final int INTAKE_MOTOR_CAN_ID = 18;
    public static final NeutralModeValue INTAKE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final CurrentLimitsConfigs INTAKE_CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)
            .withSupplyCurrentLimit(30);

    public static RollerSim intakeSim;
    public static final RollerConfig INTAKE_ROLLER_SIM_CONFIG = new RollerConfig(3)
            .setPosition(1.6, .9);

    // PID
    public static final double kP_intake = 0.041842;
    public static final double kI_intake = 0;
    public static final double kD_intake = 0;

    public static final double kS_intake = 0.34866;
    public static final double kV_intake = 0.097215;
    public static final double kA_intake = 0.0047301;
}
