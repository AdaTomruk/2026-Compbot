// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Turret motor disconnected.", AlertType.kError);
  private final Alert cancoder10tDisconnectedAlert =
      new Alert("Turret 10t CANcoder disconnected.", AlertType.kError);
  private final Alert cancoder17tDisconnectedAlert =
      new Alert("Turret 17t CANcoder disconnected.", AlertType.kError);
  private final Alert crtResolveFailedAlert =
      new Alert("Turret CRT resolve failed — position may be incorrect.", AlertType.kError);

  private double targetAngleDeg = 0.0;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);
    cancoder10tDisconnectedAlert.set(!inputs.cancoder10tConnected);
    cancoder17tDisconnectedAlert.set(!inputs.cancoder17tConnected);
    crtResolveFailedAlert.set(!inputs.crtResolveSucceeded);
  }

  public Command setAbsoluteOrientationCommand(double angleDeg) {
    return Commands.runOnce(
        () -> {
          targetAngleDeg = angleDeg;
          io.setTargetAngleDeg(targetAngleDeg);
        },
        this);
  }

  public Command setRelativeOrientationCommand(double deltaDeg) {
    return Commands.runOnce(
        () -> {
          targetAngleDeg += deltaDeg;
          io.setTargetAngleDeg(targetAngleDeg);
        },
        this);
  }

  public double getTargetAngleDeg() {
    return targetAngleDeg;
  }
}
