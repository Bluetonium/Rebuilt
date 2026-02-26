package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.utils.sim.RollerConfig;

public class ShooterConstants {
        //
        // Flywheel motor constants
        //
        public static final double FLYWHEEL_ACCELERATION = 90;
        public static final double FLYWHEEL_FORWARD_VELOCITY = -90;
        public static final double FLYWHEEL_BACKWARD_VELOCITY = 90;
        public static  InvertedValue FLYWHEEL_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

        //can id
        public static final int FLYWHEEL_MOTOR_CAN_ID = 16;
        public static final NeutralModeValue FLYWHEEL_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

        //current limits
        public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS = new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(50)// this should be like maybe 50-60, more current = more power,
                        .withSupplyCurrentLimit(30);// keep this bellow 40 ideally
        
        
        public static final double FLYWHEEL_GEAR_RATIO = 1;

        //PID values
        public static final double kP_flywheel = 0.030071;
        public static final double kI_flywheel = 0;
        public static final double kD_flywheel = 0;

        public static final double kS_flywheel = 0.17242;
        public static final double kV_flywheel = 0.12144;
        public static final double kA_flywheel = 0.0032487;

        //simulator stuff
        public static final RollerConfig FLYWHEEL_ROLLER_SIM_CONFIG = new RollerConfig(4)
                        .setPosition(1.341, .35);


        //---------------------------
        // Loader motor constants
        //---------------------------
        public static final double LOADER_ACCELERATION = 90;
        public static final double LOADER_FORWARD_VELOCITY = -10;
        public static final double LOADER_BACKWARD_VELOCITY = 10;
        public static  InvertedValue LOADER_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

        //can id
        public static final int LOADER_MOTOR_CAN_ID = 18;
        public static final NeutralModeValue LOADER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

        //current limits
        //what do these need to be??? its a x44 motor so yeah there you go. please help i dont trust Claude hes drinking all my water
        public static final CurrentLimitsConfigs LOADER_CURRENT_LIMITS = new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(50)
                        .withSupplyCurrentLimit(30);

        public static final double LOADER_GEAR_RATIO = 1;

        //PID values
        public static final double kP_loader = 0.038024;
        public static final double kI_loader = 0;
        public static final double kD_loader = 0;

        public static final double kS_loader = 0.079022;
        public static final double kV_loader = 0.12114;
        public static final double kA_loader = 0.0028202;

        //simulation stuff
        public static final RollerConfig LOADER_ROLLER_SIM_CONFIG = new RollerConfig(4)
                        .setPosition(1.35, 0.15);
}
