// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class TurretConstants {
    public static final int MOTOR_CAN_ID = 20; // TODO: assign real ID
    public static final int CANCODER_10T_CAN_ID = 21; // TODO: assign real ID
    public static final int CANCODER_17T_CAN_ID = 22; // TODO: assign real ID
    public static final double MOTOR_TO_TURRET_RATIO = 32.5833; // (46/12) x (85/10)
    public static final double CANCODER_10T_TO_TURRET = 8.5; // 85/10
    public static final double CANCODER_17T_TO_TURRET = 5.0; // 85/17
    public static final double kP = 5.0; // TODO: tune on real hardware
    public static final double kI = 0.0; // TODO: tune on real hardware
    public static final double kD = 0.1; // TODO: tune on real hardware
    public static final double MAX_VELOCITY_DEG_PER_SEC = 360.0; // TODO: tune
    public static final double MAX_ACCEL_DEG_PER_SEC_SQ = 720.0; // TODO: tune
    public static final double ZERO_OFFSET_DEG = 0.0; // TODO: calibrate mechanical zero
    public static final double CRT_MATCH_TOLERANCE_ROT = 0.05; // TODO: tune based on backlash
    public static final double CANCODER_10T_OFFSET_ROT = 0.0; // TODO: set after mechanical zeroing
    public static final double CANCODER_17T_OFFSET_ROT = 0.0; // TODO: set after mechanical zeroing
    public static final boolean SOFT_LIMIT_ENABLED = false;
    public static final double SOFT_LIMIT_MIN_DEG = -180.0;
    public static final double SOFT_LIMIT_MAX_DEG = 180.0;
    public static final double TURRET_INERTIA_KG_M2 = 0.001; // TODO: measure or estimate for sim
  }
}
