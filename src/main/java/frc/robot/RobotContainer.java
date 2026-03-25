// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.config.CANMappings;
import frc.robot.config.HopperConfig;
import frc.robot.config.IntakeConfig;
import frc.robot.config.TunerConstants;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  private Flywheel flywheel = new Flywheel();
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Hopper hopper = new Hopper();
  private Intake intake = new Intake();
  private Kicker kicker = new Kicker();
  private CommandXboxController controller = new CommandXboxController(1);
  private final SendableChooser<Command> autoChooser;
  public static boolean isExtended = false;
  public static String shooterState = "none";

  private Command shootGroup =
      Commands.parallel(new KickerCommand(kicker, KickerCommand.Position.INTAKE))
          .finallyDo(
              interrupt ->
                  CommandScheduler.getInstance()
                      .schedule(
                          Commands.runEnd(
                                  () -> hopper.move(HopperConfig.HOPPER_EXTEND_ROTATION),
                                  () -> hopper.stop(),
                                  hopper)
                              .withDeadline(Commands.waitSeconds(2))));
  private Command hopperShoot =
      Commands.repeatingSequence(
          Commands.runEnd(
                  () -> hopper.slowMove(HopperConfig.HOPPER_RETRACT_ROTATION),
                  () -> hopper.stop(),
                  hopper)
              .withDeadline(Commands.waitSeconds(.75)),
          Commands.runEnd(
                  () -> hopper.slowMove(HopperConfig.HOPPER_EXTEND_ROTATION),
                  () -> hopper.stop(),
                  hopper)
              .withDeadline(Commands.waitSeconds(.75)));

  public RobotContainer() {

    NamedCommands.registerCommand(
        "idle",
        Commands.runOnce(() -> hopper.stop(), hopper)
            .alongWith(Commands.runOnce(() -> intake.stop(), intake))
            .alongWith(Commands.runOnce(() -> kicker.stop(), kicker)));

    NamedCommands.registerCommand(
        "hopper deploy",
        Commands.runEnd(
                () -> hopper.move(HopperConfig.HOPPER_EXTEND_ROTATION), () -> hopper.stop(), hopper)
            .alongWith(Commands.run(() -> System.out.println("Hopper Deployed")))
            .withDeadline(Commands.waitSeconds(0.75)));

    NamedCommands.registerCommand(
        "hopper retract",
        Commands.runEnd(
                () -> hopper.move(HopperConfig.HOPPER_RETRACT_ROTATION),
                () -> hopper.stop(),
                hopper)
            .alongWith(Commands.run(() -> System.out.println("Hopper Retracted")))
            .withDeadline(Commands.waitSeconds(2)));

    NamedCommands.registerCommand(
        "intake",
        Commands.run(() -> intake.intake(IntakeConfig.INTAKE_INTAKE_SPEED), intake)
            .alongWith(Commands.run(() -> System.out.println("Intaking"))));

    NamedCommands.registerCommand(
        "rev shooter",
        new FlywheelCommand(flywheel, FlywheelCommand.Position.TRENCH_SHOT, drivetrain)
            .withDeadline(Commands.waitSeconds(1)));

    NamedCommands.registerCommand(
        "shoot long",
        Commands.parallel(
                new KickerCommand(kicker, KickerCommand.Position.INTAKE),
                new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE),
                new FlywheelCommand(flywheel, FlywheelCommand.Position.TRENCH_SHOT, drivetrain))
            .withDeadline(Commands.waitSeconds(8)));

    autoChooser = AutoBuilder.buildAutoChooser("Test");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private void configureBindings() {
    /* Default commands */
    // Drive
    drivetrain.setDefaultCommand(
        new DrivetrainCommand(
            drivetrain,
            DrivetrainCommand.Position.TELEOP,
            controller::getLeftX,
            controller::getLeftY,
            controller::getRightX));

    controller
        .x()
        .toggleOnTrue(
            new DrivetrainCommand(
                drivetrain,
                DrivetrainCommand.Position.AUTO_ALIGN_HUB,
                controller::getLeftX,
                controller::getLeftY,
                controller::getRightX));

    /* Controls */
    // Y = Shoot Toggle
    controller
        .y()
        .toggleOnTrue(
            (shootGroup
                .alongWith(hopperShoot)
                .alongWith(new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE))));
    // X = Intake Toggle
    controller
        .x()
        .and(() -> !shootGroup.isScheduled())
        .toggleOnTrue(new IntakeCommand(intake, IntakeCommand.Position.INTAKE));
    // Left Plus = Kicker Outtake Toggle
    controller
        .povLeft()
        .and(() -> !shootGroup.isScheduled())
        .toggleOnTrue(new KickerCommand(kicker, KickerCommand.Position.OUTTAKE));
    // Right Plus = Intake Outtake Toggle
    controller
        .povRight()
        .and(() -> !shootGroup.isScheduled())
        .toggleOnTrue(new IntakeCommand(intake, IntakeCommand.Position.OUTTAKE));
    // Right Stick Down = Extend/Retract Hopper
    controller
        .rightStick()
        .and(() -> !shootGroup.isScheduled())
        .onTrue(
            Commands.runEnd(
                    () -> hopper.move(Hopper.getRotation(isExtended)), () -> hopper.stop(), hopper)
                .withDeadline(Commands.waitSeconds(2))
                .andThen(Commands.runOnce(() -> isExtended = !isExtended)));
    // Back button = Zero Pigeon
    controller.back().onTrue(Commands.runOnce(() -> zeroPigeon()));
    // Right Bumper = Flywheel Toggle for hub shot speeds
    controller
        .rightBumper()
        .toggleOnTrue(
            new FlywheelCommand(flywheel, FlywheelCommand.Position.HUB_SHOT, drivetrain)
                .alongWith(Commands.run(() -> shooterState = "HUB")));
    // Left Bumper = Flywheel Toggle for tower shot speeds
    controller
        .leftBumper()
        .toggleOnTrue(
            new FlywheelCommand(flywheel, FlywheelCommand.Position.TOWER_SHOT, drivetrain)
                .alongWith(Commands.run(() -> shooterState = "TOWER")));
    // Left Trigger = Flywheel Toggle for trench shot speeds
    controller
        .leftTrigger()
        .toggleOnTrue(
            new FlywheelCommand(flywheel, FlywheelCommand.Position.TRENCH_SHOT, drivetrain)
                .alongWith(Commands.run(() -> shooterState = "TRENCH")));
    // Down Plus = Zero hopper
    controller.povDown().onTrue(Commands.runOnce(() -> hopper.zero(), hopper));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static void zeroPigeon() {
    Pigeon2 pigeon = new Pigeon2(CANMappings.PIGEON_CAN_ID);
    pigeon.reset();
    System.out.println("Reset Pigeon");
  }
}
