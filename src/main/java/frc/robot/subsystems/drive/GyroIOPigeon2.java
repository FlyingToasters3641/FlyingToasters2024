// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  public static final String CANbusName = "Lucas";
  private final Pigeon2 pigeon = new Pigeon2(14, CANbusName);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2(boolean phoenixDrive) {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
    if (phoenixDrive) {
      yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance().registerSignal(() -> pigeon.getYaw().getValue());
    } else {
      yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance()
              .registerSignal(() -> pigeon.getYaw().getValueAsDouble());
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
