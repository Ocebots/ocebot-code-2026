package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConfig {
  public static final double TURRET_SUPPLY_CURRENT_LIMIT = 70;
  public static final double TURRET_STATOR_CURRENT_LIMIT = 80;
  public static final double TURRET_GEAR_RATIO = 4;

  public static final double TURRET_MAX_CRUISE_VELOCITY = 3000;
  public static final double TURRET_TARGET_ACCELERATION = 500;
  public static final double TURRET_P = 0;
  public static final double TURRET_I = 0;
  public static final double TURRET_D = 0;

  public static final double TURRET_TOLERANCE = 0.005;
  public static final Translation2d ROBOT_TO_TURRET = new Translation2d(0, 0);
}
