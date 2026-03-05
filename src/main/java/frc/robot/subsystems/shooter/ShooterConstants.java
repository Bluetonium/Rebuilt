package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.utils.sim.RollerConfig;
import frc.utils.sim.RollerSim;

public class ShooterConstants {

        public static final double MOTOR_UNJAM_VELOCITY = 10;

        //-----------------------------
        // Flywheel Lead (X60)
        //-----------------------------

        public static final double FLYWHEEL_ACCELERATION = 20;
        public static double FLYWHEEL_TARGET_VELOCITY = 0;
        public static double FLYWHEEL_VELOCITY_THRESHOLD = 2;
        public static  InvertedValue FLYWHEEL_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

        //can id
        public static final int FLYWHEEL_LEAD_MOTOR_CAN_ID = 16;
        public static final NeutralModeValue FLYWHEEL_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

        //current limits
        public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS = new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(50)// this should be like maybe 50-60, more current = more power,
                        .withSupplyCurrentLimit(30);// keep this bellow 40 ideally
        
        
        public static final double FLYWHEEL_GEAR_RATIO = 1;

        //simulator stuff
        public static RollerSim FLYWHEEL_MOTOR_LEAD_SIM;
        public static final RollerConfig FLYWHEEL_MOTOR_LEAD_SIM_CONFIG = new RollerConfig(4)
                        .setPosition(1, .8)
                        .setGearRatio(1)      // match your actual gear ratio
                        .setSimMOI(0.00005);

        //PID values
        public static final double kP_flywheel = 0.054237;
        public static final double kI_flywheel = 0;
        public static final double kD_flywheel = 0;

        public static final double kS_flywheel = 0.15735;
        public static final double kV_flywheel = 0.11918;
        public static final double kA_flywheel = 0.004995;

        //-----------------------------
        // Flywheel Follower (X60)
        //-----------------------------

        //can id
        public static final int FLYWHEEL_FOLLOWER_MOTOR_CAN_ID = 17;

        //simulator stuff
        public static RollerSim FLYWHEEL_MOTOR_FOLLOWER_SIM;
        public static final RollerConfig FLYWHEEL_MOTOR_FOLLOWER_SIM_CONFIG = new RollerConfig(4)
                        .setPosition(0.85, .8)
                        .setGearRatio(1)      // match your actual gear ratio
                        .setSimMOI(0.00005);

        // If i hold left trigger, right trigger, left trigger, it stops all commands

        //-----------------------------
        // Loader Motor (X44)
        //-----------------------------

        public static final double LOADER_ACCELERATION = 45;
        public static final double LOADER_FORWARD_VELOCITY = 75;
        public static  InvertedValue LOADER_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

        //can id
        public static final int LOADER_MOTOR_CAN_ID = 19;
        public static final NeutralModeValue LOADER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

        //current limits
        //what do these need to be??? its a x44 motor so yeah there you go. please help i dont trust Claude hes drinking all my water
        public static final CurrentLimitsConfigs LOADER_CURRENT_LIMITS = new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(50)
                        .withSupplyCurrentLimit(30);

        public static final double LOADER_GEAR_RATIO = 1;

        //simulation stuff
        public static RollerSim LOADER_SIM;
        public static final RollerConfig LOADER_SIM_CONFIG = new RollerConfig(3)
                        .setPosition(1, .5);

        //PID values
        public static final double kP_loader = 0.024619;
        public static final double kI_loader = 0;
        public static final double kD_loader = 0;

        public static final double kS_loader = 0.495;
        public static final double kV_loader = 0.10723;
        public static final double kA_loader = 0.0035311;

        //-----------------------------
        // Transfer Motor (X44)
        //-----------------------------

        public static final double TRANSFER_ACCELERATION = 30;
        public static final double TRANSFER_FORWARD_VELOCITY = -50;
        public static  InvertedValue TRANSFER_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

        //can id
        public static final int TRANSFER_MOTOR_CAN_ID = 21;
        public static final NeutralModeValue TRANSFER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

        //current limits
        //what do these need to be??? its a x44 motor so yeah there you go. please help i dont trust Claude hes drinking all my water
        public static final CurrentLimitsConfigs TRANSFER_CURRENT_LIMITS = new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(50)
                        .withSupplyCurrentLimit(30);

        public static final double TRANSFER_GEAR_RATIO = 3;

        //simulation stuff
        public static RollerSim TRANSFER_SIM;
        public static final RollerConfig TRANSFER_SIM_CONFIG = new RollerConfig(3)
                        .setPosition(1.3, .5);

        //PID values
        public static final double kP_transfer = 0.12258;
        public static final double kI_transfer = 0;
        public static final double kD_transfer = 0;

        public static final double kS_transfer = 1.2135;
        public static final double kV_transfer = 0.069086;
        public static final double kA_transfer = 0.46091;
}
