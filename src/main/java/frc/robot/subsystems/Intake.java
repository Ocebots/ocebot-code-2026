package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.IntakeConfig;

public class Intake extends SubsystemBase {
  protected TalonFX intake;

  public Intake() {
    intake = new TalonFX(CANMappings.INTAKE_MOTOR_ID);
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConfig.INTAKE_SUPPLY_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConfig.INTAKE_STATOR_CURRENT_LIMIT;

    intakeConfig.Feedback.SensorToMechanismRatio = IntakeConfig.INTAKE_GEAR_RATIO;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intake.getConfigurator().apply(intakeConfig);
  }

  public void intake(double speed) {
    speed = -Math.abs(speed);
    intake.setControl(new DutyCycleOut(speed));
  }

  public void outtake(double speed) {
    speed = Math.abs(speed);
    intake.setControl(new DutyCycleOut(speed));
  }

  // Runs intake with shoot toggle, temp
  public void slowIntake(double speed) {
    speed = -Math.abs(speed);
    intake.setControl(new DutyCycleOut(speed));
  }

  public void stop() {
    intake.stopMotor();
  }
}
