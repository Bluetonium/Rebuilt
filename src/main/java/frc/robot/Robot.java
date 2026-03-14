// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Logger;

import org.littletonrobotics.junction.LoggedRobot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private final Field2d m_field = new Field2d();

    @SuppressWarnings("unused")
    private RobotSim sim;// whoopsy you forgot to make this so the sim doesnt work, i fixed it though C:

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        sim = new RobotSim();

        SendableRegistry.add(CommandScheduler.getInstance(),"Command Scheduler");
        SmartDashboard.putData(CommandScheduler.getInstance());

        // Logger.recordMetadata("ProjectName", "MyProject");
        // Logger.start();
    }

    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        // Logger.recordOutput("RobotPose", m_robotContainer.drivetrain.getPose());
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {

        SignalLogger.stop();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.drivetrain.seedFieldCentric();
        m_robotContainer.initHubPosition();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        SignalLogger.start();
        m_robotContainer.initHubPosition();
        if (m_autonomousCommand != null) {

            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void teleopPeriodic() {
        Pose2d pose = m_robotContainer.drivetrain.getState().Pose;

        // update robot position
        m_field.setRobotPose(pose);

        Translation2d end =
            pose.getTranslation().plus(
                new Translation2d(2, pose.getRotation())
            );

        Translation2d end2 =
            pose.getTranslation().plus(
                new Translation2d(4, pose.getRotation())
            );

        Translation2d end3 =
            pose.getTranslation().plus(
                new Translation2d(6, pose.getRotation())
            );

        m_field.getObject("heading").setPoses(
            pose,
            new Pose2d(end, pose.getRotation()),
            new Pose2d(end2, pose.getRotation()),
            new Pose2d(end3, pose.getRotation())
        );
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
