// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
  private Flywheel flywheel = new Flywheel();
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Hopper hopper = new Hopper();
  private Climb climb = new Climb();
  private Intake intake = new Intake();
  private Kicker kicker = new Kicker();
  private CommandXboxController controller = new CommandXboxController(0);
  private SwerveDriveState driveState = new SwerveDriveState();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // Should I move these to a config file?

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Default Commands + Drivetrain
    flywheel.setDefaultCommand(
        new FlywheelCommand(flywheel, FlywheelCommand.Position.DEFAULT_SHOT, drivetrain));

    // Drivetrain Teleop drive with controller inputs
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-controller.getLeftY() * MaxSpeed)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate)));

    // Buttons

    // Y = Shoot Toggle
    controller
        .y()
        .toggleOnTrue(
            Commands.parallel(
                new KickerCommand(kicker, KickerCommand.Position.OUTTAKE),
                new HopperCommand(hopper, HopperCommand.Position.SHOOT_RETRACT_EXTEND),
                // We may need to change this, because if controller also presses X at the same time
                // bad things could happen
                new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE)));
    // X = intake toggle
    controller.x().toggleOnTrue(new IntakeCommand(intake, IntakeCommand.Position.INTAKE));
    // Right Stick Down = Extend/Retract Hopper
    controller
        .rightStick()
        .toggleOnTrue(new HopperCommand(hopper, HopperCommand.Position.EXTEND_RETRACT));
    // Right Trigger = Climb Extend
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
  }

  public Command getAutonomousCommand() {
    // Not finished yet
    // Does withDeadline work here?
    return (Commands.run(() -> flywheel.getDefaultCommand(), flywheel))
        .withDeadline(Commands.waitSeconds(5));
  }

  public static void zeroPigeon() {
    Pigeon2 pigeon = new Pigeon2(CANMappings.PIGEON_CAN_ID);
    pigeon.reset();
    System.out.println("Reset Pigeon");
  }
}
