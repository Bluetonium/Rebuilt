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
import com.ctre.phoenix6.signals.InvertedValue;

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
    private TalonFX motor;
    private RollerSim sim;
    private final VoltageOut m_sysIdControl = new VoltageOut(0);
    private TalonFXConfiguration motorConfig;

    private MotionMagicVelocityVoltage mmVelocityVoltage = new MotionMagicVelocityVoltage(0)
            .withAcceleration(ShooterConstants.ACCELERATION);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");//TODO rename to be consistant
        builder.addDoubleProperty("Target Velocity", () -> mmVelocityVoltage.Velocity, null);
        builder.addDoubleProperty("Velocity", () -> motor.getVelocity().getValueAsDouble(), null);
    }

    // Who knows what ts does
    // I know what this does C: .... will i tell you... yes
    //this is used when tuning the mechanics, its a command you run that logs the motion of the motor while moving and then you use sysid to analize it
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("SysIdOuttake_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> motor.setControl(m_sysIdControl.withOutput(volts.in(Volts))),
                    null,
                    this));

    public Shooter() {
        motor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_CAN_ID);
        motor.setNeutralMode(ShooterConstants.SHOOTER_MOTOR_NEUTRAL_MODE);

        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//TODO maybe have this be in constants
        motorConfig.CurrentLimits = ShooterConstants.CURRENT_LIMITS;

        FeedbackConfigs feedback = motorConfig.Feedback;
        feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

        Slot0Configs slot0 = motorConfig.Slot0;
        slot0.kP = ShooterConstants.kP;
        slot0.kI = ShooterConstants.kI;
        slot0.kD = ShooterConstants.kD;
        slot0.kS = ShooterConstants.kS;
        slot0.kV = ShooterConstants.kV;
        slot0.kA = ShooterConstants.kA;

        applyConfig();

        sim = new RollerSim(ShooterConstants.ROLLER_SIM_CONFIG, RobotSim.rightView, motor.getSimState(), "Outtake");//TODO rename to be consistant
        
        SendableRegistry.add(this, "Module 0");//TODO rename?? why is this module 0
        SmartDashboard.putData(this);
    }

    private void applyConfig() {
        StatusCode status = motor.getConfigurator().apply(motorConfig);
        if (!status.isOK()) {
            DriverStation.reportWarning(
                    status.getName() + "Failed to apply configs to outtake" + status.getDescription(), false);//TODO rename to be consistant
        }
    }

    public void setup() {
        setDefaultCommand(run(() -> {
            motor.setControl(mmVelocityVoltage.withVelocity(0));
        }).withName("Outtake.Stopped"));//TODO once again rename to be consistant

        ShooterStates.setupStates();
    }

    @Override
    public void simulationPeriodic() {
        sim.simulationPeriodic();
    }

    //TODO can map these to buttons that will like never be used or like have a flag in the code you can set that enables it.
    //last year we used a smart dashboard thing... hmm perhaps it could use like a control mapped to slot 3 or whatever
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    // In the final version this should have 2 motors + 3 commands
    // 1: shoot, runs both motors
    // 2: startup, runs the flywheel to speed it up but not the loader
    // 3: unjam, runs everything in reverse

    public Command runForward() {
        return new StartEndCommand(
        () -> {
            motor.setControl(mmVelocityVoltage.withVelocity(ShooterConstants.FORWARD_VELOCITY));
        },
        () -> {
            motor.setControl(mmVelocityVoltage.withVelocity(0));
        }, this);
    }

    public Command runBackward() {
        return new StartEndCommand(
        () -> {
            motor.setControl(mmVelocityVoltage.withVelocity(ShooterConstants.BACKWARD_VELOCITY));
        },
        () -> {
            motor.setControl(mmVelocityVoltage.withVelocity(0));
        }, this);
    }
}