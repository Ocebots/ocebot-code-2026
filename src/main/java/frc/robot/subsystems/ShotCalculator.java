package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class ShotCalculator {
  public static double calculateTurretSOTM(
      Translation2d robotPosition, Translation2d robotVelocity) {
    // 1. Project future position
    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));
    // 2. Get target vector
    Translation2d toGoal = calculateGoalPosition().minus(futurePos);
    double distance = toGoal.getNorm();
    Translation2d targetDirection = toGoal.div(distance);
    // 3. Look up baseline velocity from table
    ShooterParams baseline = SHOOTER_MAP.get(distance);
    double baselineVelocity = distance / baseline.flightTime;
    // 4. Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);
    // 5. subtract robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);
    // 6. Extract results
    double turretAngle = shotVelocity.getAngle().getRotations();

    return turretAngle;
  }

  public static double calculateTurretStillShot(
      Translation2d robotPosition, Translation2d robotVelocity) {
    return 5.0;
  }

  public static double calculateTurretPass(
      Translation2d robotPosition, Translation2d robotVelocity) {
    return 5.0;
  }

  public static double calculateHoodPass(Translation2d robotPosition, Translation2d robotVelocity) {
    return 5.0;
  }

  public static double calculateFlywheelPass(
      Translation2d robotPosition, Translation2d robotVelocity) {
    return 5.0;
  }

  public static double calculateFlywheelShot(
      Translation2d robotPosition, Translation2d robotVelocity) {
    // 1. Project future position
    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));
    // 2. Get target vector
    Translation2d toGoal = calculateGoalPosition().minus(futurePos);
    double distance = toGoal.getNorm();
    Translation2d targetDirection = toGoal.div(distance);
    // 3. Look up baseline velocity from table
    ShooterParams baseline = SHOOTER_MAP.get(distance);
    double baselineVelocity = distance / baseline.flightTime;
    // 4. Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);
    // 5. THE MAGIC: subtract robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);
    // 6. Extract results
    Rotation2d turretAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();
    // 7. Use table in reverse: velocity → effective distance → RPM
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
    double requiredRpm = SHOOTER_MAP.get(effectiveDistance).rpm;
    return requiredRpm;
  }

  public static double calculateHoodShot(Translation2d robotPosition, Translation2d robotVelocity) {
    // 1. Project future position
    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));
    // 2. Get target vector
    Translation2d toGoal = calculateGoalPosition().minus(futurePos);
    double distance = toGoal.getNorm();
    Translation2d targetDirection = toGoal.div(distance);
    // 3. Look up baseline velocity from table
    ShooterParams baseline = SHOOTER_MAP.get(distance);
    double baselineVelocity = distance / baseline.flightTime;
    // 4. Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);
    // 5. THE MAGIC: subtract robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);
    // 6. Extract results
    Rotation2d turretAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();
    // 7. Use table in reverse: velocity → effective distance → hood angle
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
    double requiredHoodAngle = SHOOTER_MAP.get(effectiveDistance).hoodAngle;
    return requiredHoodAngle;
  }

  public static double velocityToEffectiveDistance(double velocity) {
    // Binary search or iterate through table to find distance
    // where (distance / ToF) = velocity
    // Most InterpolatingTreeMap implementations support inverse lookup
    // or you can build a reverse map: velocity → distance

    return VELOCITY_MAP.get(velocity);
  }

  public double calculateAdjustedRpm(double requiredVelocity) {
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
    return SHOOTER_MAP.get(effectiveDistance).rpm;
  }

  public static Translation2d calculateGoalPosition() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        return new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
      } else if (alliance.get() == DriverStation.Alliance.Red) {
        return new Translation2d(
            Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84));
      }
    }
    return null;
  }

  public record ShooterParams(double rpm, double hoodAngle, double flightTime) {}

  // Distance (feet) → RPM, Hood Angle (rotations), Time of Flight (s)
  private static final InterpolatingTreeMap<Double, ShotCalculator.ShooterParams> SHOOTER_MAP =
      new InterpolatingTreeMap<>(
          MathUtil::inverseInterpolate,
          (start, end, t) ->
              new ShooterParams(
                  MathUtil.interpolate(start.rpm, end.rpm, t),
                  MathUtil.interpolate(start.hoodAngle, end.hoodAngle, t),
                  MathUtil.interpolate(start.flightTime, end.flightTime, t)));

  static {
    SHOOTER_MAP.put(Units.feetToMeters(1.5), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(3.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(4.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(5.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(6.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(8.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(10.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(12.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(14.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(16.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(18.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
    SHOOTER_MAP.put(Units.feetToMeters(20.0), new ShotCalculator.ShooterParams(0.0, 0.0, 0.00));
  }

  private static final InterpolatingDoubleTreeMap VELOCITY_MAP = new InterpolatingDoubleTreeMap();

  static {
    VELOCITY_MAP.put(
        Units.feetToMeters(1.5) / SHOOTER_MAP.get(Units.feetToMeters(1.5)).flightTime,
        Units.feetToMeters(1.5));
    VELOCITY_MAP.put(
        Units.feetToMeters(3.0) / SHOOTER_MAP.get(Units.feetToMeters(3.0)).flightTime,
        Units.feetToMeters(3.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(4.0) / SHOOTER_MAP.get(Units.feetToMeters(4.0)).flightTime,
        Units.feetToMeters(4.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(5.0) / SHOOTER_MAP.get(Units.feetToMeters(5.0)).flightTime,
        Units.feetToMeters(5.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(6.0) / SHOOTER_MAP.get(Units.feetToMeters(6.0)).flightTime,
        Units.feetToMeters(6.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(8.0) / SHOOTER_MAP.get(Units.feetToMeters(8.0)).flightTime,
        Units.feetToMeters(8.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(10.0) / SHOOTER_MAP.get(Units.feetToMeters(10.0)).flightTime,
        Units.feetToMeters(10.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(12.0) / SHOOTER_MAP.get(Units.feetToMeters(12.0)).flightTime,
        Units.feetToMeters(12.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(14.0) / SHOOTER_MAP.get(Units.feetToMeters(14.0)).flightTime,
        Units.feetToMeters(14.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(16.0) / SHOOTER_MAP.get(Units.feetToMeters(16.0)).flightTime,
        Units.feetToMeters(16.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(18.0) / SHOOTER_MAP.get(Units.feetToMeters(18.0)).flightTime,
        Units.feetToMeters(18.0));
    VELOCITY_MAP.put(
        Units.feetToMeters(20.0) / SHOOTER_MAP.get(Units.feetToMeters(20.0)).flightTime,
        Units.feetToMeters(20.0));
  }

  public static final double latencyCompensation = 0.0;
}
