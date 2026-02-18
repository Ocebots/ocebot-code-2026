// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.*;
import frc.robot.config.CANMappings;
import frc.robot.config.TunerConstants;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  private Turret turret = new Turret();
  private Hood hood = new Hood();
  private Flywheel flywheel = new Flywheel();
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Hopper hopper = new Hopper();
  private Climb climb = new Climb();
  private Intake intake = new Intake();
  private Spindexer spindexer = new Spindexer();
  private Kicker kicker = new Kicker();
  private CommandXboxController controller = new CommandXboxController(0);
  private SwerveDriveState driveState = new SwerveDriveState();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Drive
    drivetrain.setDefaultCommand(
        new DrivetrainCommand(
            drivetrain,
            DrivetrainCommand.Position.TELEOP,
            controller.getLeftX(),
            controller.getLeftY(),
            controller.getRightX()));

    // Zero
    controller.x().onTrue(Commands.runOnce(RobotContainer::zeroPigeon));

    // Intake
    controller.leftTrigger().whileTrue(new IntakeCommand(intake, IntakeCommand.Position.INTAKE));

    // Shoot
    controller.rightTrigger().whileTrue(new KickerCommand(kicker, KickerCommand.Position.INTAKE));
    controller
        .rightTrigger()
        .whileTrue(new FlywheelCommand(flywheel, FlywheelCommand.Position.SHOOT_SIMPLE));

    // Climb
    controller.a().whileTrue(new ClimbCommand(climb, ClimbCommand.Position.CLIMB));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static void zeroPigeon() {
    Pigeon2 pigeon = new Pigeon2(CANMappings.PIGEON_CAN_ID);
    pigeon.reset();
    System.out.println("Reset Pigeon");
  }
}
