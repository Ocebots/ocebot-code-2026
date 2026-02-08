package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.IntakeConfig;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  public static enum Position {
    INTAKE,
    OUTTAKE
  }

  private Intake subsystem;
  private IntakeCommand.Position pose;

  public IntakeCommand(Intake subsystem, IntakeCommand.Position pose) {
    this.pose = pose;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case INTAKE:
        subsystem.intake(IntakeConfig.INTAKE_INTAKE_SPEED);
        break;

      case OUTTAKE:
        subsystem.outtake(IntakeConfig.INTAKE_OUTTAKE_SPEED);
        break;

      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
