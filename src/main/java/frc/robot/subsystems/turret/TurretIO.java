// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public boolean cancoder10tConnected = false;
    public boolean cancoder17tConnected = false;
    public double turretPositionDeg = 0.0;
    public double turretVelocityDegPerSec = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
    public double cancoder10tPositionRot = 0.0;
    public double cancoder17tPositionRot = 0.0;
    public boolean crtResolveSucceeded = false;
    public double crtResolvedAngleDeg = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setTargetAngleDeg(double angleDeg) {}

  public default void triggerCRTResolve() {}
}
