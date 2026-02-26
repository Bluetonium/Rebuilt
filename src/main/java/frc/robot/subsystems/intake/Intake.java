package frc.robot.subsystems.intake;

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


public class Intake extends SubsystemBase{

    private TalonFX intakeMotor;
    private RollerSim intakeSim;
    private final VoltageOut intakeSysIdControl = new VoltageOut(0);
    private TalonFXConfiguration intakeMotorConfig;

    private MotionMagicVelocityVoltage intakeVelocityVoltage = new MotionMagicVelocityVoltage(0).withAcceleration(IntakeConstants.INTAKE_ACCELERATION);

    //the intake dropper motor variables and stuff
    private TalonFX intakeDropperMotor;
    private RollerSim intakeDropperSim;
    private final VoltageOut intakeDropperSysIdControl = new VoltageOut(0);
    private TalonFXConfiguration intakeDropperMotorConfig;

    private MotionMagicVelocityVoltage intakeDropperVelocityVoltage = new MotionMagicVelocityVoltage(0).withAcceleration(IntakeConstants.INTAKE_DROPPER_ACCELERATION);

    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Intake");

        builder.addDoubleProperty("Intake Target Velocity", () -> intakeVelocityVoltage.Velocity,null);
        builder.addDoubleProperty("Intake Velocity", () -> intakeMotor.getVelocity().getValueAsDouble(),null);

        builder.addDoubleProperty("Intake Dropper Target Velocity", () -> intakeDropperVelocityVoltage.Velocity,null);
        builder.addDoubleProperty("Intake Dropper Velocity", () -> intakeDropperMotor.getVelocity().getValueAsDouble(),null);
        //
        // You stopped here Kaiden, don't forget where you stopped that would be bad
        // continue from here ->
        //
    }

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("SysIdIntake", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> intakeMotor.setControl(intakeSysIdControl.withOutput(volts.in(Volts))),
                    null,
                    this));

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
        intakeMotor.setNeutralMode(IntakeConstants.INTAKE_MOTOR_NEUTRAL_MODE);
        
        intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeMotorConfig.CurrentLimits = IntakeConstants.INTAKE_CURRENT_LIMITS;

        FeedbackConfigs feedback = intakeMotorConfig.Feedback;
        feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEAR_RATIO;

        Slot0Configs slot0 = intakeMotorConfig.Slot0;
        slot0.kP = IntakeConstants.kP_intake;
        slot0.kI = IntakeConstants.kI_intake;
        slot0.kD = IntakeConstants.kD_intake;
        slot0.kS = IntakeConstants.kS_intake;
        slot0.kV = IntakeConstants.kV_intake;
        slot0.kA = IntakeConstants.kA_intake;

        applyConfig();

        intakeSim = new RollerSim(IntakeConstants.INTAKE_ROLLER_SIM_CONFIG,RobotSim.rightView,intakeMotor.getSimState(),"Intake");
        
        SendableRegistry.add(this,"Module 1" );//TODO rename? idk it says this in shooter.java
        SmartDashboard.putData(this);
    }

    private void applyConfig() {
        StatusCode status = intakeMotor.getConfigurator().apply(intakeMotorConfig);
        if (!status.isOK()) {
            DriverStation.reportWarning(
                    status.getName() + "Failed to apply configs to outtake" + status.getDescription(), false);//TODO rename to be consistant
        }
    }

    public void setup() {

        setDefaultCommand(run(() -> {
            intakeMotor.setControl(intakeVelocityVoltage.withVelocity(0));
        }).withName("Intake.Stopped"));//TODO once again rename to be consistant

        IntakeStates.setupStates();
    }

    @Override
    public void simulationPeriodic() {
        intakeSim.simulationPeriodic();
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
            intakeMotor.setControl(intakeVelocityVoltage.withVelocity(IntakeConstants.INTAKE_FORWARD_VELOCITY));
            
        },
        () -> {
            intakeMotor.setControl(intakeVelocityVoltage.withVelocity(0));
        }, this).withName("IntakeForward");
    }

    public Command runBackward() {
        return new StartEndCommand(
        () -> {
            intakeMotor.setControl(intakeVelocityVoltage.withVelocity(IntakeConstants.INTAKE_BACKWARD_VELOCITY));
        },
        () -> {
            intakeMotor.setControl(intakeVelocityVoltage.withVelocity(0));
        }, this).withName("IntakeReverse");
    }
}
