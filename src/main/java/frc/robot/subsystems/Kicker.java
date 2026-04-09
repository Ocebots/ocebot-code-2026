package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.KickerConfig;

@Logged
public class Kicker extends SubsystemBase {
  protected TalonFX kickerRight;
  protected TalonFX kickerLeft;

  public Kicker() {
    kickerRight = new TalonFX(CANMappings.KICKER_RIGHT_MOTOR_ID);
    kickerLeft = new TalonFX(CANMappings.KICKER_LEFT_MOTOR_ID);
    TalonFXConfiguration kickerRightConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerLeftConfig = new TalonFXConfiguration();

    kickerRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    kickerRightConfig.CurrentLimits.SupplyCurrentLimit =
        KickerConfig.KICKER_RIGHT_SUPPLY_CURRENT_LIMIT;
    kickerRightConfig.CurrentLimits.StatorCurrentLimit =
        KickerConfig.KICKER_RIGHT_STATOR_CURRENT_LIMIT;
    kickerLeftConfig.CurrentLimits.SupplyCurrentLimit =
        KickerConfig.KICKER_LEFT_SUPPLY_CURRENT_LIMIT;
    kickerLeftConfig.CurrentLimits.StatorCurrentLimit =
        KickerConfig.KICKER_LEFT_STATOR_CURRENT_LIMIT;

    kickerRightConfig.Feedback.SensorToMechanismRatio = KickerConfig.KICKER_RIGHT_GEAR_RATIO;
    kickerRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kickerLeftConfig.Feedback.SensorToMechanismRatio = KickerConfig.KICKER_LEFT_GEAR_RATIO;
    kickerLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kickerRight.getConfigurator().apply(kickerRightConfig);
    kickerLeft.getConfigurator().apply(kickerLeftConfig);
  }

  public void intake(double speed) {
    speed = Math.abs(speed);
    kickerRight.setControl(new VoltageOut(speed));
    kickerLeft.setControl(new VoltageOut(-speed));
  }

  public void intake(double rightSpeed, double leftSpeed) {
    rightSpeed = Math.abs(rightSpeed);
    leftSpeed = Math.abs(leftSpeed);
    kickerRight.setControl(new VoltageOut(rightSpeed));
    kickerLeft.setControl(new VoltageOut(-leftSpeed));
  }

  public void outtake(double speed) {
    speed = Math.abs(speed);
    kickerRight.setControl(new VoltageOut(-speed));
    kickerLeft.setControl(new VoltageOut(speed));
  }

  public void outtake(double rightSpeed, double leftSpeed) {
    rightSpeed = Math.abs(rightSpeed);
    leftSpeed = Math.abs(leftSpeed);
    kickerRight.setControl(new VoltageOut(-rightSpeed));
    kickerLeft.setControl(new VoltageOut(leftSpeed));
  }

  public void stop() {
    kickerRight.stopMotor();
    kickerLeft.stopMotor();
  }
}
