package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.HopperConfig;

public class Hopper extends SubsystemBase {
  private TalonFX leftHopper;
  private TalonFX rightHopper;

  public Hopper() {

    leftHopper = new TalonFX(CANMappings.LEFT_HOPPER_MOTOR_ID);
    rightHopper = new TalonFX(CANMappings.RIGHT_HOPPER_MOTOR_ID);

    TalonFXConfiguration leftHopperConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightHopperConfig = new TalonFXConfiguration();

    leftHopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftHopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftHopperConfig.CurrentLimits.StatorCurrentLimit =
        HopperConfig.LEFT_HOPPER_STATOR_CURRENT_LIMIT;
    leftHopperConfig.CurrentLimits.SupplyCurrentLimit =
        HopperConfig.LEFT_HOPPER_SUPPLY_CURRENT_LIMIT;
    ;

    rightHopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightHopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightHopperConfig.CurrentLimits.StatorCurrentLimit =
        HopperConfig.RIGHT_HOPPER_STATOR_CURRENT_LIMIT;
    ;
    rightHopperConfig.CurrentLimits.SupplyCurrentLimit =
        HopperConfig.RIGHT_HOPPER_SUPPLY_CURRENT_LIMIT;
    ;

    leftHopperConfig.MotionMagic.MotionMagicCruiseVelocity =
        HopperConfig.LEFT_HOPPER_MAX_CRUISE_VELOCITY;
    rightHopperConfig.MotionMagic.MotionMagicCruiseVelocity =
        HopperConfig.RIGHT_HOPPER_MAX_CRUISE_VELOCITY;
    leftHopperConfig.MotionMagic.MotionMagicAcceleration =
        HopperConfig.LEFT_HOPPER_TARGET_ACCELERATION;
    rightHopperConfig.MotionMagic.MotionMagicAcceleration =
        HopperConfig.RIGHT_HOPPER_TARGET_ACCELERATION;

    leftHopperConfig.Slot0.kP = HopperConfig.LEFT_HOPPER_P;
    leftHopperConfig.Slot0.kI = HopperConfig.LEFT_HOPPER_I;
    leftHopperConfig.Slot0.kD = HopperConfig.LEFT_HOPPER_D;

    rightHopperConfig.Slot0.kP = HopperConfig.RIGHT_HOPPER_P;
    rightHopperConfig.Slot0.kI = HopperConfig.RIGHT_HOPPER_I;
    rightHopperConfig.Slot0.kD = HopperConfig.RIGHT_HOPPER_D;

    leftHopperConfig.Feedback.SensorToMechanismRatio =
        HopperConfig.LEFT_HOPPER_GEAR_RATIO; // gear ratio
    rightHopperConfig.Feedback.SensorToMechanismRatio = HopperConfig.RIGHT_HOPPER_GEAR_RATIO;

    leftHopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightHopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leftHopper.getConfigurator().apply(leftHopperConfig);
    rightHopper.getConfigurator().apply(rightHopperConfig);

    // follower = new Follower(CANMappings.K_Hopper_LEFT_ID, false);
  }

  public void extend(double rotation) {
    leftHopper.setControl(new MotionMagicVoltage(rotation));
    rightHopper.setControl(new MotionMagicVoltage(rotation));
  }

  public void retract(double rotation) {
    leftHopper.setControl(new MotionMagicVoltage(rotation));
    rightHopper.setControl(new MotionMagicVoltage(rotation));
  }

  public void directionalExtend(double speed) {
    leftHopper.setControl(new DutyCycleOut(speed));
    rightHopper.setControl(new DutyCycleOut(speed));
  }

  public void directionalRetract(double speed) {
    leftHopper.setControl(new DutyCycleOut(speed));
    rightHopper.setControl(new DutyCycleOut(speed));
  }

  public void stop() {
    leftHopper.stopMotor();
    rightHopper.stopMotor();
  }

  public void zero() {
    leftHopper.setPosition(0.0);
    rightHopper.setPosition(0.0);
  }

  public boolean atPosition() {
    return (Math.abs(leftHopper.getClosedLoopError().getValueAsDouble())
            <= HopperConfig.HOPPER_TOLERANCE)
        && (Math.abs(rightHopper.getClosedLoopError().getValueAsDouble())
            <= HopperConfig.HOPPER_TOLERANCE);
  }
}
