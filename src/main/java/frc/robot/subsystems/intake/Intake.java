package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
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


public class Intake extends SubsystemBase{

    private TalonFX motor;
    private RollerSim sim;
    private final VoltageOut m_sysIdControl = new VoltageOut(0);
    private TalonFXConfiguration motorConfig;

    private MotionMagicVelocityVoltage mmVelocityVoltage = new MotionMagicVelocityVoltage(0).withAcceleration(IntakeConstants.ACCELERATION);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Intake");
        builder.addDoubleProperty("Target Velocity", () -> mmVelocityVoltage.Velocity,null);
        builder.addDoubleProperty("Velocity", () -> motor.getVelocity().getValueAsDouble(),null);
    }

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("SysIdIntake", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> motor.setControl(m_sysIdControl.withOutput(volts.in(Volts))),
                    null,
                    this));

    public Intake() {
        motor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
        motor.setNeutralMode(IntakeConstants.INTAKE_MOTOR_NEUTRAL_MODE);
        
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.CurrentLimits = IntakeConstants.CURRENT_LIMITS;

        FeedbackConfigs feedback = motorConfig.Feedback;
        feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;

        Slot1Configs slot1 = motorConfig.Slot1;
        slot1.kP = IntakeConstants.kP;
        slot1.kI = IntakeConstants.kI;
        slot1.kD = IntakeConstants.kD;
        slot1.kS = IntakeConstants.kS;
        slot1.kV = IntakeConstants.kV;
        slot1.kA = IntakeConstants.kA;

        applyConfig();

        sim = new RollerSim(IntakeConstants.ROLLER_SIM_CONFIG,RobotSim.rightView,motor.getSimState(),"Intake");
        
        SendableRegistry.add(this,"Module 1" );//TODO rename? idk it says this in shooter.java
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
        }).withName("Intake.Stopped"));//TODO once again rename to be consistant

        IntakeStates.setupStates();
    }

    @Override
    public void simulationPeriodic() {
        sim.simulationPeriodic();
    }


    //sys id stuff
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
    //------

    public Command runForward() {
        return new StartEndCommand(
        () -> {
            motor.setControl(mmVelocityVoltage.withVelocity(IntakeConstants.FORWARD_VELOCITY));
        },
        () -> {
            motor.setControl(mmVelocityVoltage.withVelocity(0));
        }, this);
    }

    public Command runBackward() {
        return new StartEndCommand(
        () -> {
            motor.setControl(mmVelocityVoltage.withVelocity(IntakeConstants.BACKWARD_VELOCITY));
        },
        () -> {
            motor.setControl(mmVelocityVoltage.withVelocity(0));
        }, this);
    }
}
