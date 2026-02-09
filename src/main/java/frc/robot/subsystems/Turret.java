package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.CANMappings;
import frc.robot.config.TurretConfig;

public class Turret extends SubsystemBase {
  protected TalonFX turret;
  private double turretScoreRotation;

  public Turret() {
    turret = new TalonFX(CANMappings.TURRET_MOTOR_ID);

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretConfig.CurrentLimits.StatorCurrentLimit = TurretConfig.TURRET_STATOR_CURRENT_LIMIT;

    turretConfig.CurrentLimits.SupplyCurrentLimit = TurretConfig.TURRET_SUPPLY_CURRENT_LIMIT;

    turretConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConfig.TURRET_MAX_CRUISE_VELOCITY;
    turretConfig.MotionMagic.MotionMagicAcceleration = TurretConfig.TURRET_TARGET_ACCELERATION;

    turretConfig.Slot0.kP = TurretConfig.TURRET_P;
    turretConfig.Slot0.kI = TurretConfig.TURRET_I;
    turretConfig.Slot0.kD = TurretConfig.TURRET_D;

    turretConfig.Feedback.SensorToMechanismRatio = TurretConfig.TURRET_GEAR_RATIO;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turret.getConfigurator().apply(turretConfig);
  }

  public void rotate(double rotation) {
    turret.setControl(new MotionMagicVoltage(rotation));
  }

  public void directionalRotate(double speed) {
    turret.setControl(new DutyCycleOut(speed));
  }

  public void stop() {
    turret.stopMotor();
  }

  public void zero() {
    turret.setPosition(0.0);
  }

  public boolean atPosition() {
    return (Math.abs(turret.getClosedLoopError().getValueAsDouble())
        <= TurretConfig.TURRET_TOLERANCE);
  }

  final PositionVoltage m_request = new PositionVoltage(0);

  boolean flipping = false;

  public void moveToApriltag() {
    if (LimelightHelpers.getTV("limelight") && !flipping) {
      double currentPosition = turret.getPosition().getValueAsDouble();
      double tx = -LimelightHelpers.getTX("limelight");
      double errorInRotations = (tx / 360.0) * TurretConfig.TURRET_GEAR_RATIO;
      double targetPositon = currentPosition + errorInRotations;
      turret.setControl(m_request.withPosition(targetPositon));
    }
  }

  public void flip() {
    if (turret.getPosition().getValueAsDouble() >= 2) {
      flipping = true;
      stop();
      turret.setControl(m_request.withPosition(-1.9));
      new WaitCommand(2);
      flipping = false;
    }
    if (turret.getPosition().getValueAsDouble() <= -2) {
      flipping = true;
      stop();
      turret.setControl(m_request.withPosition(1.9));
      new WaitCommand(2);
      flipping = false;
    }
  }

  public boolean shouldFlip() {
    if (turret.getPosition().getValueAsDouble() >= 2
        || turret.getPosition().getValueAsDouble() <= -2) {
      return true;
    } else {
      return false;
    }
  }
}
