// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.config.VisionConfig.photonPoseEstimatorForward;
import static frc.robot.config.VisionConfig.result;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.TunerConstants;
import frc.robot.config.VisionConfig;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Epilogue.bind(this);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Optional<EstimatedRobotPose> visionEst =
        VisionConfig.photonPoseEstimatorForward.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) {
      visionEst = VisionConfig.photonPoseEstimatorForward.estimateLowestAmbiguityPose(result);
    } else {
      drivetrain.addVisionMeasurement(
          photonPoseEstimatorForward
              .estimateAverageBestTargetsPose(result)
              .get()
              .estimatedPose
              .toPose2d(),
          visionEst.get().timestampSeconds);
    }
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
