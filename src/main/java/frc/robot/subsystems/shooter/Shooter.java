package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotSim;
import frc.utils.sim.RollerSim;

public class Shooter extends SubsystemBase {
    //Kraken S44

    //@Getter
    private TalonFX flywheelMotor;
    private TalonFX loaderMotor;
    
    private RollerSim flywheelSim;
    private final VoltageOut flywheelSysIdControl = new VoltageOut(0);
    private TalonFXConfiguration flywheelMotorConfig;

    //left off here, make everything for loader motor
    private RollerSim loaderSim;
    private final VoltageOut loaderSysIdControl = new VoltageOut(0);
    private TalonFXConfiguration loaderMotorConfig;

    private MotionMagicVelocityVoltage flywheelVelocityVoltage = new MotionMagicVelocityVoltage(0)
            .withAcceleration(ShooterConstants.FLYWHEEL_ACCELERATION);

    private MotionMagicVelocityVoltage loaderVelocityVoltage = new MotionMagicVelocityVoltage(0)
    .withAcceleration(ShooterConstants.LOADER_ACCELERATION);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");//TODO rename to be consistant

        builder.addDoubleProperty("Flywheel Target Velocity", () -> flywheelVelocityVoltage.Velocity, null);
        builder.addDoubleProperty("Flywheel Velocity", () -> flywheelMotor.getVelocity().getValueAsDouble(), null);

        builder.addDoubleProperty("Loader Target Velocity", () -> loaderVelocityVoltage.Velocity,null);
        builder.addDoubleProperty("Loader Velocity", () -> loaderMotor.getVelocity().getValueAsDouble(), null);
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
                    (volts) -> flywheelMotor.setControl(flywheelSysIdControl.withOutput(volts.in(Volts))),
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


    public Shooter() {
        flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_CAN_ID);
        flywheelMotor.setNeutralMode(ShooterConstants.FLYWHEEL_MOTOR_NEUTRAL_MODE);

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
        //-----------------------------------
        loaderMotor = new TalonFX(ShooterConstants.LOADER_MOTOR_CAN_ID);
        loaderMotor.setNeutralMode(ShooterConstants.LOADER_MOTOR_NEUTRAL_MODE);

        loaderMotorConfig = new TalonFXConfiguration();
        loaderMotorConfig.MotorOutput.Inverted = ShooterConstants.LOADER_MOTOR_INVERTED;
        loaderMotorConfig.CurrentLimits = ShooterConstants.LOADER_CURRENT_LIMITS;

        FeedbackConfigs loaderFeedback = loaderMotorConfig.Feedback;
        loaderFeedback.SensorToMechanismRatio = ShooterConstants.LOADER_GEAR_RATIO;

        Slot0Configs loaderSlot0 = flywheelMotorConfig.Slot0;
        loaderSlot0.kP = ShooterConstants.kP_loader;
        loaderSlot0.kI = ShooterConstants.kI_loader;
        loaderSlot0.kD = ShooterConstants.kD_loader;
        loaderSlot0.kS = ShooterConstants.kS_loader;
        loaderSlot0.kV = ShooterConstants.kV_loader;
        loaderSlot0.kA = ShooterConstants.kA_loader;
        //-----------------------------------
        applyConfig();

        flywheelSim = new RollerSim(ShooterConstants.FLYWHEEL_ROLLER_SIM_CONFIG, RobotSim.rightView, flywheelMotor.getSimState(), "Flywheel Motor");//TODO rename to be consistant -- i made it flywheel, is this what you meant?
        loaderSim = new RollerSim(ShooterConstants.LOADER_ROLLER_SIM_CONFIG, RobotSim.rightView, loaderMotor.getSimState(), "Loader Motor");

        SendableRegistry.add(this, "Shooter");
        SmartDashboard.putData(this);
    }

    private void applyConfig() {
        StatusCode flywheelStatus = flywheelMotor.getConfigurator().apply(flywheelMotorConfig);
        StatusCode loaderStatus = loaderMotor.getConfigurator().apply(loaderMotorConfig);

        if (!flywheelStatus.isOK()) {
            DriverStation.reportWarning(
                    flywheelStatus.getName() + "Failed to apply configs to flywheel" + flywheelStatus.getDescription(), false);
        }
        
        //i assume this is how it works, idk i just copied it so that each motor has their own warning
        if (!loaderStatus.isOK()) {
            DriverStation.reportWarning(
                loaderStatus.getName() + "Failed to apply configs to loader" + loaderStatus.getDescription(), false);
        }
    }

    public void setup() {
        setDefaultCommand(run(() -> {
            flywheelMotor.setControl(flywheelVelocityVoltage.withVelocity(0));
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
        }).withName("Outtake.Stopped"));//TODO once again rename to be consistant

        ShooterStates.setupStates();
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.simulationPeriodic();
        loaderSim.simulationPeriodic();
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

    // In the final version this should have 2 motors + 3 commands
    // 1: shoot, runs both motors
    // 2: startup, runs the flywheel to speed it up but not the loader
    // 3: unjam, runs everything in reverse
    
    public Command runForward() {
        return new StartEndCommand(
        () -> {
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(ShooterConstants.LOADER_FORWARD_VELOCITY));
            flywheelMotor.setControl(flywheelVelocityVoltage.withVelocity(ShooterConstants.FLYWHEEL_FORWARD_VELOCITY));
        },
        () -> {
            loaderMotor.setControl(loaderVelocityVoltage.withVelocity(0));
            flywheelMotor.setControl(flywheelVelocityVoltage.withVelocity(0));
        }, this).withName("ShooterForward");
    }

    public Command runBackward() {
        return new StartEndCommand(
        () -> {
            flywheelMotor.setControl(flywheelVelocityVoltage.withVelocity(ShooterConstants.FLYWHEEL_BACKWARD_VELOCITY));
        },
        () -> {
            flywheelMotor.setControl(flywheelVelocityVoltage.withVelocity(0));
        }, this).withName("ShooterReverse");
    }
}