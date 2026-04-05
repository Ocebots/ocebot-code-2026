// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.FlywheelCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  // last time we injected a synthetic vision measurement (seconds, FPGA time)
  private double m_lastSimVisionTime = 0.0;
  // whether we've injected a one-time offset vision measurement for testing
  private boolean m_injectedOffset = false;
  private Pose2d robotPose = new Pose2d();
  private final RobotContainer m_robotContainer;
  private final CommandSwerveDrivetrain drivetrain;
  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
  StructArrayPublisher<Pose2d> arrayPublisher =
      NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

  public Robot() {
    m_robotContainer = new RobotContainer();
    drivetrain = m_robotContainer.getDrivetrain();
    Epilogue.bind(this);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("Flywheel State", FlywheelCommand.flywheelState);
    SmartDashboard.putString("Drivetrain State", DrivetrainCommand.drivetrainState);
    SmartDashboard.putString("Hopper State", Hopper.hopperState);
    SmartDashboard.putString("Intake State", Intake.intakeState);
    SmartDashboard.putString("Kicker State", KickerCommand.kickerState);
    SmartDashboard.putBoolean(
        "Flywheel On",
        !(FlywheelCommand.flywheelState.equals("none")
            || FlywheelCommand.flywheelState.equals("Stopped")));
    try {
      var pose = drivetrain.getState().Pose;
      SmartDashboard.putNumber("OdometryX", pose.getX());
      SmartDashboard.putNumber("OdometryY", pose.getY());
      SmartDashboard.putNumber("OdometryRotDeg", pose.getRotation().getDegrees());
      SmartDashboard.putNumber("SimVisionLastInject", m_lastSimVisionTime);
    } catch (Exception ignored) {
    }
    robotPose = drivetrain.getState().Pose;
    publisher.set(robotPose);
    arrayPublisher.set(new Pose2d[] {robotPose});
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    double now = Timer.getFPGATimestamp();
    if (now - m_lastSimVisionTime >= 0.1) {
      m_lastSimVisionTime = now;
      try {
        var truePose = drivetrain.getState().Pose;
        // After 3 seconds, inject a single offset measurement (+1m X) to observe correction
        if (!m_injectedOffset && now > 3.0) {
          Pose2d offsetPose =
              new Pose2d(truePose.getX() + 1.0, truePose.getY(), truePose.getRotation());
          drivetrain.addVisionMeasurement(offsetPose, now);
          m_injectedOffset = true;
        } else {
          drivetrain.addVisionMeasurement(truePose, now);
        }
      } catch (Exception e) {
        DriverStation.reportError("Sim vision injection failed: " + e.getMessage(), false);
      }
    }
  }
}
