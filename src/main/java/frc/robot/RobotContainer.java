// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.TurretCommand;
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

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller
        .a()
        .toggleOnTrue(new TurretCommand(turret, TurretCommand.Position.STILL_SHOT, drivetrain));
    controller
        .b()
        .toggleOnTrue(new TurretCommand(turret, TurretCommand.Position.MOVING_SHOT, drivetrain));
    controller.x().toggleOnTrue(new TurretCommand(turret, TurretCommand.Position.PASS , drivetrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
