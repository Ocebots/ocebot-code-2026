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
    // Y = Shoot Toggle
    controller.y().toggleOnTrue(new KickerCommand(kicker, KickerCommand.Position.OUTTAKE));
    // X = intake toggle
    controller.x().toggleOnTrue(new IntakeCommand(intake, IntakeCommand.Position.INTAKE));
    // Right Stick Down = Extend/Retract Hopper
    /*
    controller.rightStick().onTrue(
            new HopperCommand(hopper, HopperCommand.Position.EXTEND),
            new HopperCommand(hopper, HopperCommand.Position.RETRACT),
            () -> hopper.isExtended() // Fix this method later!!!
    );
    */
    // Right Trigger = Climb Extend
    controller.rightTrigger().toggleOnTrue(new ClimbCommand(climb, ClimbCommand.Position.CLIMB));
    // Left Trigger = Climb Extend
    controller.leftTrigger().toggleOnTrue(new ClimbCommand(climb, ClimbCommand.Position.UNCLIMB));
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
