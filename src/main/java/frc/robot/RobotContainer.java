// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
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
  private Climb climb = new Climb();
  private Intake intake = new Intake();
  private Kicker kicker = new Kicker();
  private CommandXboxController controller = new CommandXboxController(1);
  private SwerveDriveState driveState = new SwerveDriveState();
  private final SendableChooser<Command> autoChooser;
  public static boolean isExtended = false;

  private Command shootGroup =
      Commands.parallel(
              new KickerCommand(kicker, KickerCommand.Position.INTAKE),
              Commands.runEnd(
                      () -> hopper.slowMove(HopperConfig.HOPPER_RETRACT_ROTATION),
                      () -> hopper.stop(),
                      hopper)
                  .withDeadline(Commands.waitSeconds(2)),
              new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE))
          .finallyDo(
              interrupt ->
                  CommandScheduler.getInstance()
                      .schedule(
                          Commands.runEnd(
                                  () -> hopper.move(HopperConfig.HOPPER_EXTEND_ROTATION),
                                  () -> hopper.stop(),
                                  hopper)
                              .withDeadline(Commands.waitSeconds(2))));

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
            .withDeadline(Commands.waitSeconds(1)));

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
    // Flywheel
    flywheel.setDefaultCommand(
        new FlywheelCommand(flywheel, FlywheelCommand.Position.DEFAULT_SHOT, drivetrain));

    /* Controls */
    // Y = Shoot Toggle
    controller.y().toggleOnTrue(shootGroup);
    // X = Intake Toggle
    controller
        .x()
        .and(() -> !shootGroup.isScheduled())
        .toggleOnTrue(new IntakeCommand(intake, IntakeCommand.Position.INTAKE));

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

    //    // Right Trigger = Climb Retract
    controller
        .rightTrigger()
        .whileTrue(new ClimbCommand(climb, ClimbCommand.Position.DIRECTIONAL_CLIMB));
    // Left Trigger = Climb Extend
    controller
        .leftTrigger()
        .whileTrue(new ClimbCommand(climb, ClimbCommand.Position.DIRECTIONAL_UNCLIMB));
    // Back button = Zero Pigeon gyro
    controller.back().onTrue(Commands.runOnce(() -> zeroPigeon()));
    // Left Bumper = Flywheel Toggle for hub shot speeds
    controller
        .leftBumper()
        .toggleOnTrue(new FlywheelCommand(flywheel, FlywheelCommand.Position.HUB_SHOT, drivetrain));
    // Right Bumper = Flywheel Toggle for tower shot speeds
    controller
        .rightBumper()
        .toggleOnTrue(
            new FlywheelCommand(flywheel, FlywheelCommand.Position.TOWER_SHOT, drivetrain));
    // A = Flywheel Toggle of calculated shots
    controller
        .a()
        .toggleOnTrue(
            new FlywheelCommand(flywheel, FlywheelCommand.Position.CALCULATED_SHOT, drivetrain));
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
