// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
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
import frc.robot.config.HopperConfig;
import frc.robot.config.IntakeConfig;
import frc.robot.config.TunerConstants;
import frc.robot.helpers.ShotCalculator;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  private Flywheel flywheel = new Flywheel();
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Hopper hopper = new Hopper();
  private Intake intake = new Intake();
  private Kicker kicker = new Kicker();
  private CommandXboxController controller = new CommandXboxController(1);
  private CommandXboxController operator = new CommandXboxController(2);
  private final SendableChooser<Command> autoChooser;
  public static boolean isExtended = false;
  private SwerveDrivetrain.SwerveDriveState driveState = drivetrain.getState();

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
                    .withDeadline(Commands.waitSeconds(.75)))
            .withDeadline(Commands.waitSeconds(5)));

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

    NamedCommands.registerCommand(
        "short shoot",
        Commands.parallel(
                new KickerCommand(kicker, KickerCommand.Position.INTAKE),
                new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE),
                new FlywheelCommand(flywheel, FlywheelCommand.Position.TRENCH_SHOT, drivetrain))
            .withDeadline(Commands.waitSeconds(3)));

    NamedCommands.registerCommand(
        "hopper down",
        Commands.runEnd(
                () -> hopper.move(HopperConfig.HOPPER_EXTEND_ROTATION), () -> hopper.stop(), hopper)
            .withDeadline(Commands.waitSeconds(0.5)));

    NamedCommands.registerCommand(
        "hub shot",
        Commands.parallel(
                new KickerCommand(kicker, KickerCommand.Position.INTAKE),
                new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE),
                new FlywheelCommand(flywheel, FlywheelCommand.Position.HUB_SHOT, drivetrain))
            .withDeadline(Commands.waitSeconds(10)));

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

    /* Controls */
    controller
        .rightTrigger()
        .toggleOnTrue(
            new DrivetrainCommand(
                drivetrain,
                DrivetrainCommand.Position.AUTO_ALIGN_HUB,
                controller::getLeftX,
                controller::getLeftY,
                controller::getRightX));
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
    controller.back().onTrue(Commands.runOnce(drivetrain::zeroPigeon));
    // Right Bumper = Flywheel Toggle for hub shot speeds
    controller
        .rightBumper()
        .toggleOnTrue(new FlywheelCommand(flywheel, FlywheelCommand.Position.HUB_SHOT, drivetrain));
    // Left Bumper = Flywheel Toggle for trench shot speeds
    controller
        .leftBumper()
        .toggleOnTrue(
            new FlywheelCommand(flywheel, FlywheelCommand.Position.TRENCH_SHOT, drivetrain));
    // Left Trigger = Flywheel Toggle for calculated shot speeds
    controller
        .leftTrigger()
        .toggleOnTrue(
            new FlywheelCommand(flywheel, FlywheelCommand.Position.CALCULATED_SHOT, drivetrain));
    // Down Plus = Zero hopper
    controller.povDown().onTrue(Commands.runOnce(() -> hopper.zero(), hopper));

    /* Operator */
    operator
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> drivetrain.setPose(ShotCalculator.calculateLeftCornerRobotPosition())));
    operator
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> drivetrain.setPose(ShotCalculator.calculateRightCornerRobotPosition())));
  }

  public Command getAutonomousCommand() {
    return 
    new FlywheelCommand(flywheel, FlywheelCommand.Position.HUB_SHOT, drivetrain)
            .withDeadline(Commands.waitSeconds(3)).andThen(
        Commands.parallel(
                new KickerCommand(kicker, KickerCommand.Position.INTAKE),
                new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE),
                new FlywheelCommand(flywheel, FlywheelCommand.Position.HUB_SHOT, drivetrain))
            .withDeadline(Commands.waitSeconds(8)));
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }
}
