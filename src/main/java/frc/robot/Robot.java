// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

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

        Logger.recordMetadata("Rebuilt", "MyProject");
        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // write logs to USB
        }
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }

    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        Pose2d robotPose = m_robotContainer.drivetrain.getState().Pose;

        double length = 6.0;

        Translation2d frontTranslation =
            robotPose.getTranslation().plus(
                new Translation2d(length, 0).rotateBy(robotPose.getRotation())
            );

        Pose2d frontPose = new Pose2d(frontTranslation, robotPose.getRotation());

        Trajectory headingTrajectory = new Trajectory(
            List.of(
                new Trajectory.State(0, 0, 0, robotPose, 0),
                new Trajectory.State(0, 0, 0, frontPose, 0)
            )
        );

        // target trajectory
        Translation2d targetTranslation =
            robotPose.getTranslation().plus(
                new Translation2d(length, 0).rotateBy(m_robotContainer.autoaim.getAngleToHub().plus(Rotation2d.fromDegrees(180)))
            );


        Pose2d targetPose = new Pose2d(targetTranslation, m_robotContainer.autoaim.getAngleToHub().plus(Rotation2d.fromDegrees(180)));

        Trajectory targetTrajectory = new Trajectory(
            List.of(
                new Trajectory.State(0, 0, 0, robotPose, 0),
                new Trajectory.State(0, 0, 0, targetPose, 0)
            )
        );

        Pose2d targetPoint = new Pose2d(m_robotContainer.getHubX(), 4.034, new Rotation2d());

        Logger.recordOutput("Annotations/TargetPoint", targetPoint);
        Logger.recordOutput("Annotations/Pose", robotPose);
        Logger.recordOutput("Annotations/Heading", headingTrajectory);
        Logger.recordOutput("Annotations/targetHeading", targetTrajectory);
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
    }

    @Override
    public void teleopPeriodic() {
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
