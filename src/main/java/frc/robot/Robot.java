// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private Pose2d robotPose = new Pose2d();
  private final RobotContainer m_robotContainer;
  private final CommandSwerveDrivetrain drivetrain;
  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
  Field2d field = new Field2d();

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
    SmartDashboard.putNumber("Gyro Yaw", drivetrain.getPigeon2().getRotation2d().getDegrees());
    SmartDashboard.putBoolean(
        "Flywheel On",
        !(FlywheelCommand.flywheelState.equals("none")
            || FlywheelCommand.flywheelState.equals("Stopped")));
    robotPose = drivetrain.getState().Pose;
    publisher.set(robotPose);
    field.setRobotPose(robotPose);
    SmartDashboard.putData("Field", field);
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
  public void simulationPeriodic() {}
}
