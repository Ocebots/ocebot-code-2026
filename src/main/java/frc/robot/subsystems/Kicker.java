package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.KickerConfig;

public class Kicker extends SubsystemBase {
  protected TalonFX kickerFront;
  protected TalonFX kickerBack;

  public Kicker() {
    kickerFront = new TalonFX(CANMappings.KICKER_FRONT_MOTOR_ID);
    kickerBack = new TalonFX(CANMappings.KICKER_BACK_MOTOR_ID);
    TalonFXConfiguration kickerFrontConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerBackConfig = new TalonFXConfiguration();

    kickerFrontConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerFrontConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerBackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerBackConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    kickerFrontConfig.CurrentLimits.SupplyCurrentLimit =
        KickerConfig.KICKER_FRONT_SUPPLY_CURRENT_LIMIT;
    kickerFrontConfig.CurrentLimits.StatorCurrentLimit =
        KickerConfig.KICKER_FRONT_STATOR_CURRENT_LIMIT;
    kickerBackConfig.CurrentLimits.SupplyCurrentLimit =
        KickerConfig.KICKER_BACK_SUPPLY_CURRENT_LIMIT;
    kickerBackConfig.CurrentLimits.StatorCurrentLimit =
        KickerConfig.KICKER_BACK_STATOR_CURRENT_LIMIT;

    kickerFrontConfig.Slot0.kP = KickerConfig.KICKER_P;
    kickerFrontConfig.Slot0.kI = KickerConfig.KICKER_I;
    kickerFrontConfig.Slot0.kD = KickerConfig.KICKER_D;
    kickerFrontConfig.Slot0.kS = KickerConfig.KICKER_S;
    kickerFrontConfig.Slot0.kV = KickerConfig.KICKER_V;
    kickerFrontConfig.Slot0.kA = KickerConfig.KICKER_A;

    kickerFrontConfig.Feedback.SensorToMechanismRatio = KickerConfig.KICKER_FRONT_GEAR_RATIO;
    kickerFrontConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerFrontConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kickerBackConfig.Feedback.SensorToMechanismRatio = KickerConfig.KICKER_BACK_GEAR_RATIO;
    kickerBackConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerBackConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kickerFront.getConfigurator().apply(kickerFrontConfig);
    kickerBack.getConfigurator().apply(kickerBackConfig);
  }

  public void intake(double speed) {
    speed = Math.abs(speed);
    kickerFront.setControl(new MotionMagicVelocityVoltage(-speed));
    kickerBack.setControl(new MotionMagicVelocityVoltage(speed));
  }

  public void intake(double frontSpeed, double backSpeed) {
    frontSpeed = Math.abs(frontSpeed);
    backSpeed = Math.abs(backSpeed);
    kickerFront.setControl(new MotionMagicVelocityVoltage(-frontSpeed));
    kickerBack.setControl(new MotionMagicVelocityVoltage(backSpeed));
  }

  public void intakeDutyCycleOut(double speed) {
    speed = Math.abs(speed);
    kickerFront.setControl(new DutyCycleOut(-speed));
    kickerBack.setControl(new DutyCycleOut(speed));
  }

  public void intakeDutyCycleOut(double frontSpeed, double backSpeed) {
    frontSpeed = Math.abs(frontSpeed);
    backSpeed = Math.abs(backSpeed);
    kickerFront.setControl(new DutyCycleOut(-frontSpeed));
    kickerBack.setControl(new DutyCycleOut(backSpeed));
  }

  public void outtake(double speed) {
    speed = Math.abs(speed);
    kickerFront.setControl(new DutyCycleOut(speed));
    kickerBack.setControl(new DutyCycleOut(-speed));
  }

  public void outtake(double frontSpeed, double backSpeed) {
    frontSpeed = Math.abs(frontSpeed);
    backSpeed = Math.abs(backSpeed);
    kickerFront.setControl(new DutyCycleOut(frontSpeed));
    kickerBack.setControl(new DutyCycleOut(-backSpeed));
  }

  public void stop() {
    kickerFront.stopMotor();
    kickerBack.stopMotor();
  }
}
