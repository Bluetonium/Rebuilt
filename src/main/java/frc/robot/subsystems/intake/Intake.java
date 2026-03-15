package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotSim;
import frc.utils.sim.ArmConfig;
import frc.utils.sim.ArmSim;
import frc.utils.sim.RollerSim;



public class Intake extends SubsystemBase{


    MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private double m_targetAngle = IntakeConstants.INTAKE_DROPPER_INITIAL_ANGLE;
    private boolean m_toggleState = false;

    private TalonFX intakeMotor;
    private final VoltageOut intakeSysIdControl = new VoltageOut(0);
    private TalonFXConfiguration intakeMotorConfig;

    private MotionMagicVelocityVoltage intakeVelocityVoltage = new MotionMagicVelocityVoltage(0).withAcceleration(IntakeConstants.INTAKE_ACCELERATION);

    //the intake dropper motor variables and stuff
    private TalonFX intakeDropperMotor;
    private final VoltageOut intakeDropperSysIdControl = new VoltageOut(0);
    private TalonFXConfiguration intakeDropperMotorConfig;

    private MotionMagicVelocityVoltage intakeDropperVelocityVoltage = new MotionMagicVelocityVoltage(0).withAcceleration(IntakeConstants.INTAKE_DROPPER_ACCELERATION);

    private boolean m_runRoller = false;
    
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

    private final SysIdRoutine IntakeSysIdRoutine = new SysIdRoutine(
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
    
    private final SysIdRoutine IntakeDropperSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("SysIdIntakeDropper", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> intakeDropperMotor.setControl(intakeDropperSysIdControl.withOutput(volts.in(Volts))),
                    null,
                    this));

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
        intakeMotor.setNeutralMode(IntakeConstants.INTAKE_MOTOR_NEUTRAL_MODE);
        
        intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeMotorConfig.CurrentLimits = IntakeConstants.INTAKE_CURRENT_LIMITS;

        FeedbackConfigs intakeFeedback = intakeMotorConfig.Feedback;
        intakeFeedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEAR_RATIO; //make this constant after kaiden

        Slot0Configs intakeSlot0 = intakeMotorConfig.Slot0;
        intakeSlot0.kP = IntakeConstants.kP_intake;
        intakeSlot0.kI = IntakeConstants.kI_intake;
        intakeSlot0.kD = IntakeConstants.kD_intake;
        intakeSlot0.kS = IntakeConstants.kS_intake;
        intakeSlot0.kV = IntakeConstants.kV_intake;
        intakeSlot0.kA = IntakeConstants.kA_intake;
        //---------------------------------------
        intakeDropperMotor = new TalonFX(IntakeConstants.INTAKE_DROPPER_MOTOR_CAN_ID);
        intakeDropperMotor.setNeutralMode(IntakeConstants.INTAKE_DROPPER_MOTOR_NEUTRAL_MODE);
        
        intakeDropperMotorConfig = new TalonFXConfiguration();
        intakeDropperMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeDropperMotorConfig.CurrentLimits = IntakeConstants.INTAKE_DROPPER_CURRENT_LIMITS;

        intakeDropperMotor.setPosition(IntakeConstants.INTAKE_DROPPER_INITIAL_ANGLE / 360.0);

        FeedbackConfigs intakeDropperfeedback = intakeDropperMotorConfig.Feedback;
        intakeDropperfeedback.SensorToMechanismRatio = IntakeConstants.INTAKE_DROPPER_GEAR_RATIO;

        Slot0Configs intakeDropperSlot0 = intakeDropperMotorConfig.Slot0;
        intakeDropperSlot0.kP = IntakeConstants.kP_dropper;
        intakeDropperSlot0.kI = IntakeConstants.kI_dropper;
        intakeDropperSlot0.kD = IntakeConstants.kD_dropper;
        intakeDropperSlot0.kS = IntakeConstants.kS_dropper;
        intakeDropperSlot0.kV = IntakeConstants.kV_dropper;
        intakeDropperSlot0.kA = IntakeConstants.kA_dropper;

        intakeDropperMotorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_DROPPER_CRUISE_VELOCITY;
        intakeDropperMotorConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.INTAKE_DROPPER_ACCELERATION;

        applyConfig();

        IntakeConstants.intakeDropperArmSim = new ArmSim(
            IntakeConstants.INTAKE_DROPPER_SIM_CONFIG,
            RobotSim.rightView,  // your existing Mechanism2d
            intakeDropperMotor.getSimState(),
            "IntakeDropper"
        );
        IntakeConstants.intakeSim = new RollerSim(IntakeConstants.INTAKE_ROLLER_SIM_CONFIG.setMount(IntakeConstants.intakeDropperArmSim),RobotSim.rightView,intakeMotor.getSimState(),"Intake Motor");

        SendableRegistry.add(this,"Intake" );
        SmartDashboard.putData(this);
    }

    private void applyConfig() {
        StatusCode intakeStatus = intakeMotor.getConfigurator().apply(intakeMotorConfig);
        StatusCode intakeDropperStatus = intakeDropperMotor.getConfigurator().apply(intakeDropperMotorConfig);

        if (!intakeStatus.isOK()) {
            DriverStation.reportWarning(
                    intakeStatus.getName() + "Failed to apply configs to intake" + intakeStatus.getDescription(), false);
        }

        if (!intakeDropperStatus.isOK()) {
            DriverStation.reportWarning(
                    intakeDropperStatus.getName() + "Failed to apply configs to intake dropper" + intakeDropperStatus.getDescription(), false);
        }
    }

    public void setup() {

        setDefaultCommand(run(() -> {
            intakeMotor.setControl(intakeVelocityVoltage.withVelocity(
                m_runRoller ? IntakeConstants.INTAKE_FORWARD_VELOCITY : 0
            ));
            intakeDropperMotor.setControl(m_motionMagicRequest.withPosition(m_targetAngle / 360.0));
        }).withName("Intake.Stopped"));//TODO once again rename to be consistant

        IntakeStates.setupStates();
    }

    @Override
    public void simulationPeriodic() {
        IntakeConstants.intakeSim.simulationPeriodic();
        IntakeConstants.intakeDropperArmSim.simulationPeriodic();    
    }


    //sys id stuff
    public Command intakeSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return IntakeSysIdRoutine.quasistatic(direction);
    }

    public Command intakeSysIdDynamic(SysIdRoutine.Direction direction) {
        return IntakeSysIdRoutine.dynamic(direction);
    }

    //sys id for dropper
    public Command intakeDropperSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return IntakeDropperSysIdRoutine.quasistatic(direction);
    }

    public Command intakeDropperSysIdDynamic(SysIdRoutine.Direction direction) {
        return IntakeDropperSysIdRoutine.dynamic(direction);
    }
    //------

    public Command runForward() {
        return new StartEndCommand(
        () -> {
            intakeMotor.setControl(intakeVelocityVoltage.withVelocity(IntakeConstants.INTAKE_FORWARD_VELOCITY));
            
        },
        () -> {
            intakeMotor.setControl(intakeVelocityVoltage.withVelocity(0));
        }).withName("IntakeForward");
    }

    public Command runForwardAuton() {
        return Commands.startEnd(
            () -> m_runRoller = true,
            () -> m_runRoller = false
        ).withName("IntakeForwardAuton");
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

    // we gotta add the braking to this !!!!!!!!!
    /*public Command toggleAngle(double angleA, double angleB) {
        return Commands.runOnce(() -> {
            m_toggleState = !m_toggleState;
            m_targetAngle = m_toggleState ? angleB : angleA;
        }).withName("IntakeToggleAngle");
    }*/

    public Command moveDown() {
        return Commands.runOnce(() -> {
            m_targetAngle = IntakeConstants.INTAKE_DOWN_ANGLE;
            intakeDropperMotor.setNeutralMode(NeutralModeValue.Coast);
        }).withName("IntakeMoveDown");
    }

    public Command moveUp() {
        return Commands.runOnce(() -> {
            m_targetAngle = IntakeConstants.INTAKE_UP_ANGLE;
            intakeDropperMotor.setNeutralMode(NeutralModeValue.Brake);
        }).withName("IntakeMoveUp");
    }

    public Command intakeFunnel() {
        return Commands.runOnce(() -> {
            m_targetAngle = IntakeConstants.INTAKE_FUNNEL_ANGLE;
            intakeDropperMotor.setNeutralMode(NeutralModeValue.Brake);
        }).withName("IntakeFunnel");
    }
}
