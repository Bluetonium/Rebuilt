package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotSim;
import frc.robot.RobotStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.utils.sim.RollerSim;

public class Shooter extends SubsystemBase {
    //Kraken S44

    //@Getter
    private TalonFX flywheelLeadMotor;
    private TalonFX flywheelFollowerMotor;

    private TalonFX loaderMotor;
    private TalonFX transferMotor;

    private DoubleSupplier distanceSupplier;

    
    private final VoltageOut flywheelSysIdControl = new VoltageOut(0);
    private TalonFXConfiguration flywheelMotorConfig;

    private final VoltageOut loaderSysIdControl = new VoltageOut(0);
    private TalonFXConfiguration loaderMotorConfig;

    private final VoltageOut transferSysIdControl = new VoltageOut(0);
    private TalonFXConfiguration transferMotorConfig;

    private MotionMagicVelocityVoltage flywheelVelocityVoltage = new MotionMagicVelocityVoltage(0)
            .withAcceleration(ShooterConstants.FLYWHEEL_ACCELERATION);

    private MotionMagicVelocityVoltage loaderVelocityVoltage = new MotionMagicVelocityVoltage(0)
            .withAcceleration(ShooterConstants.LOADER_ACCELERATION);

    private MotionMagicVelocityVoltage transferVelocityVoltage = new MotionMagicVelocityVoltage(0)
            .withAcceleration(ShooterConstants.TRANSFER_ACCELERATION);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");//TODO rename to be consistant

        builder.addDoubleProperty("Flywheel Target Velocity", () -> flywheelVelocityVoltage.Velocity, null);
        builder.addDoubleProperty("Flywheel Lead Velocity", () -> flywheelLeadMotor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("Flywheel Follower Velocity", () -> flywheelFollowerMotor.getVelocity().getValueAsDouble(), null);

        builder.addDoubleProperty("Loader Target Velocity", () -> loaderVelocityVoltage.Velocity,null);
        builder.addDoubleProperty("Loader Velocity", () -> loaderMotor.getVelocity().getValueAsDouble(), null);

        builder.addDoubleProperty("Transfer Target Velocity", () -> transferVelocityVoltage.Velocity,null);
        builder.addDoubleProperty("Transfer Velocity", () -> transferMotor.getVelocity().getValueAsDouble(), null);

        builder.addDoubleProperty("Distance to Hub", distanceSupplier::getAsDouble, null);
    }


    //this is used when tuning the mechanics, its a command you run that logs the motion of the motor while moving and then you use sysid to analize it
    private final SysIdRoutine flywheelSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("SysIdShooter", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> flywheelLeadMotor.setControl(flywheelSysIdControl.withOutput(volts.in(Volts))),
                    null,
                    this));

    private final SysIdRoutine loaderSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                (state) -> SignalLogger.writeString("SysIdLoader", state.toString())),
        new SysIdRoutine.Mechanism(
                (volts) -> loaderMotor.setControl(loaderSysIdControl.withOutput(volts.in(Volts))),
                null,
                this));


     private final SysIdRoutine transferSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                (state) -> SignalLogger.writeString("SysIdTransfer", state.toString())),
        new SysIdRoutine.Mechanism(
                (volts) -> transferMotor.setControl(transferSysIdControl.withOutput(volts.in(Volts))),
                null,
                this));

    public Shooter(DoubleSupplier distanceSupplier) {
        // Flywheel Setup
        this.distanceSupplier = distanceSupplier;

        flywheelLeadMotor = new TalonFX(ShooterConstants.FLYWHEEL_LEAD_MOTOR_CAN_ID);
        flywheelLeadMotor.setNeutralMode(ShooterConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE);

        flywheelFollowerMotor = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_CAN_ID);
        flywheelFollowerMotor.setNeutralMode(ShooterConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE);

        flywheelMotorConfig = new TalonFXConfiguration();
        flywheelMotorConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_MOTOR_INVERTED;//yay it's in constants now
        flywheelMotorConfig.CurrentLimits = ShooterConstants.FLYWHEEL_CURRENT_LIMITS;

        FeedbackConfigs flywheelFeedback = flywheelMotorConfig.Feedback;
        flywheelFeedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;

        Slot0Configs flywheelSlot0 = flywheelMotorConfig.Slot0;
        flywheelSlot0.kP = ShooterConstants.kP_flywheel;
        flywheelSlot0.kI = ShooterConstants.kI_flywheel;
        flywheelSlot0.kD = ShooterConstants.kD_flywheel;
        flywheelSlot0.kS = ShooterConstants.kS_flywheel;
        flywheelSlot0.kV = ShooterConstants.kV_flywheel;
        flywheelSlot0.kA = ShooterConstants.kA_flywheel;

        // Loader Setup

        loaderMotor = new TalonFX(ShooterConstants.LOADER_MOTOR_CAN_ID);
        loaderMotor.setNeutralMode(ShooterConstants.LOADER_MOTOR_NEUTRAL_MODE);

        loaderMotorConfig = new TalonFXConfiguration();
        loaderMotorConfig.MotorOutput.Inverted = ShooterConstants.LOADER_MOTOR_INVERTED;
        loaderMotorConfig.CurrentLimits = ShooterConstants.LOADER_CURRENT_LIMITS;

        FeedbackConfigs loaderFeedback = loaderMotorConfig.Feedback;
        loaderFeedback.SensorToMechanismRatio = ShooterConstants.LOADER_GEAR_RATIO;

        Slot0Configs loaderSlot0 = loaderMotorConfig.Slot0;
        loaderSlot0.kP = ShooterConstants.kP_loader;
        loaderSlot0.kI = ShooterConstants.kI_loader;
        loaderSlot0.kD = ShooterConstants.kD_loader;
        loaderSlot0.kS = ShooterConstants.kS_loader;
        loaderSlot0.kV = ShooterConstants.kV_loader;
        loaderSlot0.kA = ShooterConstants.kA_loader;

        // Transfer Setup

        transferMotor = new TalonFX(ShooterConstants.TRANSFER_MOTOR_CAN_ID);
        transferMotor.setNeutralMode(ShooterConstants.TRANSFER_MOTOR_NEUTRAL_MODE);

        transferMotorConfig = new TalonFXConfiguration();
        transferMotorConfig.MotorOutput.Inverted = ShooterConstants.TRANSFER_MOTOR_INVERTED;
        transferMotorConfig.CurrentLimits = ShooterConstants.TRANSFER_CURRENT_LIMITS;

        FeedbackConfigs transferFeedback = transferMotorConfig.Feedback;
        transferFeedback.SensorToMechanismRatio = ShooterConstants.TRANSFER_GEAR_RATIO;

        Slot0Configs transferSlot0 = transferMotorConfig.Slot0;
        transferSlot0.kP = ShooterConstants.kP_transfer;
        transferSlot0.kI = ShooterConstants.kI_transfer;
        transferSlot0.kD = ShooterConstants.kD_transfer;
        transferSlot0.kS = ShooterConstants.kS_transfer;
        transferSlot0.kV = ShooterConstants.kV_transfer;
        transferSlot0.kA = ShooterConstants.kA_transfer;

        // -------------------------------------------------------
        
        applyConfig();

        flywheelFollowerMotor.setControl(new Follower(ShooterConstants.FLYWHEEL_LEAD_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));

        ShooterConstants.FLYWHEEL_MOTOR_LEAD_SIM = new RollerSim(ShooterConstants.FLYWHEEL_MOTOR_LEAD_SIM_CONFIG, RobotSim.rightView, flywheelLeadMotor.getSimState(), "Flywheel Lead Motor");
        ShooterConstants.FLYWHEEL_MOTOR_FOLLOWER_SIM = new RollerSim(ShooterConstants.FLYWHEEL_MOTOR_FOLLOWER_SIM_CONFIG, RobotSim.rightView, flywheelFollowerMotor.getSimState(), "Flywheel Follower Motor");

        ShooterConstants.LOADER_SIM = new RollerSim(ShooterConstants.LOADER_SIM_CONFIG, RobotSim.rightView, loaderMotor.getSimState(), "Loader Motor");

        ShooterConstants.TRANSFER_SIM = new RollerSim(ShooterConstants.TRANSFER_SIM_CONFIG, RobotSim.rightView, transferMotor.getSimState(), "Transfer Motor");

        SendableRegistry.add(this, "Shooter");
        SmartDashboard.putData(this);
    }

    private void applyConfig() {
        StatusCode flywheelStatus = flywheelLeadMotor.getConfigurator().apply(flywheelMotorConfig);
        StatusCode flywheelFollowerStatus = flywheelFollowerMotor.getConfigurator().apply(flywheelMotorConfig);

        StatusCode loaderStatus = loaderMotor.getConfigurator().apply(loaderMotorConfig);
        StatusCode transferStatus = transferMotor.getConfigurator().apply(transferMotorConfig);

        if (!flywheelStatus.isOK()) {
            DriverStation.reportWarning(
                flywheelStatus.getName() + "Failed to apply configs to flywheel lead " + flywheelStatus.getDescription(), false);
        }

        if (!flywheelFollowerStatus.isOK()) {
            DriverStation.reportWarning(
                flywheelFollowerStatus.getName() + "Failed to apply configs to flywheel follower " + flywheelFollowerStatus.getDescription(), false);
        }
        
        if (!loaderStatus.isOK()) {
            DriverStation.reportWarning(
                loaderStatus.getName() + "Failed to apply configs to loader " + loaderStatus.getDescription(), false);
        }

        if (!transferStatus.isOK()) {
            DriverStation.reportWarning(
                transferStatus.getName() + "Failed to apply configs to transfer " + transferStatus.getDescription(), false);
        }
    }

    public void setup() {
        setDefaultCommand(run(() -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(0));
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
            transferMotor.setControl(transferVelocityVoltage.withVelocity(0));
        }).withName("Outtake.Stopped"));

        ShooterStates.setupStates();

        SmartDashboard.putNumber("StaticShooterVelocityCenter", staticShooterVelocityCenter);
    }

    @Override
    public void simulationPeriodic() {
        ShooterConstants.FLYWHEEL_MOTOR_LEAD_SIM.simulationPeriodic();
        ShooterConstants.FLYWHEEL_MOTOR_FOLLOWER_SIM.simulationPeriodic();

        ShooterConstants.LOADER_SIM.simulationPeriodic();
        ShooterConstants.TRANSFER_SIM.simulationPeriodic();

    }

    //TODO can map these to buttons that will like never be used or like have a flag in the code you can set that enables it.
    //last year we used a smart dashboard thing... hmm perhaps it could use like a control mapped to slot 3 or whatever
    public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return flywheelSysIdRoutine.quasistatic(direction);
    }

    public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
        return flywheelSysIdRoutine.dynamic(direction);
    }


    public Command loaderSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return loaderSysIdRoutine.quasistatic(direction);
    }

    public Command loaderSysIdDynamic(SysIdRoutine.Direction direction) {
        return loaderSysIdRoutine.dynamic(direction);
    }


    public Command transferSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return transferSysIdRoutine.quasistatic(direction);
    }

    public Command transferSysIdDynamic(SysIdRoutine.Direction direction) {
        return transferSysIdRoutine.dynamic(direction);
    }

    // In the final version this should have 2 motors + 3 commands
    // 1: shoot, runs both motors
    // 2: startup, runs the flywheel to speed it up but not the loader
    // 3: unjam, runs everything in reverse
    
    public Command runFlywheel() {
        return run(() -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(ShooterConstants.FLYWHEEL_TARGET_VELOCITY));
        }).finallyDo(() -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(0));
        }).withName("ShooterForward");
    }

    public Command unjamShooter() {
        return new StartEndCommand(
        () -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(ShooterConstants.MOTOR_UNJAM_VELOCITY));
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(ShooterConstants.MOTOR_UNJAM_VELOCITY));
            transferMotor.setControl(transferVelocityVoltage.withVelocity(ShooterConstants.MOTOR_UNJAM_VELOCITY));
        },
        () -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(0));
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
            transferMotor.setControl(transferVelocityVoltage.withVelocity(0));
        }, this).withName("ShooterUnjam");
    }

    public Command runLoader() {
        return new StartEndCommand(
        () -> {
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(ShooterConstants.LOADER_FORWARD_VELOCITY));
            transferMotor.setControl(transferVelocityVoltage.withVelocity(ShooterConstants.TRANSFER_FORWARD_VELOCITY));
        },
        () -> {
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
            transferMotor.setControl(transferVelocityVoltage.withVelocity(0));
        }, this).withName("LoaderForward");
    }

    //how come here we do run(() -> { }).finallyDo instead of a new StartEndCommand like above? - kaiden
    public Command runFlywheelAndLoader() {
        return run(() -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(ShooterConstants.FLYWHEEL_TARGET_VELOCITY));
            if (atTargetSpeed()) {
                loaderMotor.setControl(loaderVelocityVoltage.withVelocity(ShooterConstants.LOADER_FORWARD_VELOCITY));
                transferMotor.setControl(transferVelocityVoltage.withVelocity(ShooterConstants.TRANSFER_FORWARD_VELOCITY));
            } else {
                loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
                transferMotor.setControl(transferVelocityVoltage.withVelocity(0));
            }
        }).finallyDo(() -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(0));
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
            transferMotor.setControl(transferVelocityVoltage.withVelocity(0));
        }).withName("ShooterAndLoaderForward");
    }

    public Command constantVelocityRunFlywheelAndLoaderCenter() {
        return run(() -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(staticShooterVelocityCenter));
            if (atConstantTargetSpeedCenter()) {
                loaderMotor.setControl(loaderVelocityVoltage.withVelocity(ShooterConstants.LOADER_FORWARD_VELOCITY));
                transferMotor.setControl(transferVelocityVoltage.withVelocity(ShooterConstants.TRANSFER_FORWARD_VELOCITY));
            } else {
                loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
                transferMotor.setControl(transferVelocityVoltage.withVelocity(0));
            }
        }).finallyDo(() -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(0));
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
            transferMotor.setControl(transferVelocityVoltage.withVelocity(0));
        }).withName("ConstantVelocityShooterAndLoaderForwardCenter");
    }

    public Command constantVelocityRunFlywheelAndLoaderSide() {
        return run(() -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(staticShooterVelocitySide));
            if (atConstantTargetSpeedSide()) {
                loaderMotor.setControl(loaderVelocityVoltage.withVelocity(ShooterConstants.LOADER_FORWARD_VELOCITY));
                transferMotor.setControl(transferVelocityVoltage.withVelocity(ShooterConstants.TRANSFER_FORWARD_VELOCITY));
            } else {
                loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
                transferMotor.setControl(transferVelocityVoltage.withVelocity(0));
            }
        }).finallyDo(() -> {
            flywheelLeadMotor.setControl(flywheelVelocityVoltage.withVelocity(0));
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
            transferMotor.setControl(transferVelocityVoltage.withVelocity(0));
        }).withName("ConstantVelocityShooterAndLoaderForwardSide");
    }

    private static final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    private static double staticShooterVelocityCenter = 61; // 61
    private static double staticShooterVelocitySide = 70;

    private static boolean useStaticVelocity = false; // 75.93 inches 1.9286m (center hub to end of climber) | 57.93 inches 1.471m from center of hub

    static {
        //shooterMap.put(2.328, 60.0);
        //shooterMap.put(3.037, 65.0);
        //shooterMap.put(3.575, 70.0);
        //shooterMap.put(4.103, 80.0);
        shooterMap.put(1.79, 58.0);
        shooterMap.put(1.97, 58.0);
        shooterMap.put(2.16, 55.0);
        shooterMap.put(2.32, 56.0);
        shooterMap.put(2.82, 60.0);
        shooterMap.put(3.00, 70.0);
        shooterMap.put(3.33, 73.0);
        shooterMap.put(3.44, 76.0);
        shooterMap.put(3.55, 79.0);
        shooterMap.put(3.60, 85.0);
        shooterMap.put(3.86, 88.0);
    }

    @Override
    public void periodic() {
        staticShooterVelocityCenter = SmartDashboard.getNumber("StaticShooterVelocityCenter", staticShooterVelocityCenter);

        if(useStaticVelocity) {
            ShooterConstants.FLYWHEEL_TARGET_VELOCITY = staticShooterVelocityCenter;
        } else {
            double distance = distanceSupplier.getAsDouble();
            ShooterConstants.FLYWHEEL_TARGET_VELOCITY = shooterMap.get(distance);
        }

        flywheelLeadMotor.getSimState().setSupplyVoltage(12);
    }

    public boolean atTargetSpeed() {
        return Math.abs(flywheelLeadMotor.getVelocity().getValueAsDouble() - ShooterConstants.FLYWHEEL_TARGET_VELOCITY) <= ShooterConstants.FLYWHEEL_VELOCITY_THRESHOLD;
    }

    public boolean atConstantTargetSpeedCenter() {
        return Math.abs(flywheelLeadMotor.getVelocity().getValueAsDouble() - staticShooterVelocityCenter) <= ShooterConstants.FLYWHEEL_VELOCITY_THRESHOLD;
    }

    public boolean atConstantTargetSpeedSide() {
        return Math.abs(flywheelLeadMotor.getVelocity().getValueAsDouble() - staticShooterVelocitySide) <= ShooterConstants.FLYWHEEL_VELOCITY_THRESHOLD;
    }
}