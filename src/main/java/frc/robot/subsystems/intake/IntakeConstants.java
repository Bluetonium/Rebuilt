package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.utils.sim.RollerConfig;

public class IntakeConstants {

    
    public static final double ACCELERATION = 90;
    public static final double FORWARD_VELOCITY = -5;
    public static final double BACKWARD_VELOCITY = 5;

    public static final int INTAKE_MOTOR_CAN_ID = 17;
    public static final NeutralModeValue INTAKE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)

            .withSupplyCurrentLimit(30);
            /* 
            Keep this bellow 40 ideally, this is how much power it should be able to take
            from the PDH, 60 is too much (PS all of the supply currents of everything
            should ideally be < 120) 
            */

    public static final double GEAR_RATIO = 1;

    // PID
    //bro just stole last years pid values lmao
    public static final double kP = 0.038024;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.079022;
    public static final double kV = 0.12114;
    public static final double kA = 0.0028202;

    public static final RollerConfig ROLLER_SIM_CONFIG = new RollerConfig(4)
            .setPosition(2, .7);
}
