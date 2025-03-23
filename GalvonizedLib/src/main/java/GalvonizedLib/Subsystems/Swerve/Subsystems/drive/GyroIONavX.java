// Copyright 2021-2025 FRC 6328
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

package GalvonizedLib.Subsystems.Swerve.Subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) Drive.ODOMETRY_FREQUENCY);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> pitchPositionQueue;
  private final Queue<Double> rollPositionQueue;

  private final Queue<Double> yawTimestampQueue;
  private final String CANBusName;

  public GyroIONavX(String CANBusName) {
    this.CANBusName = CANBusName;
    yawTimestampQueue = PhoenixOdometryThread.getInstance(this.CANBusName).makeTimestampQueue();
    yawPositionQueue =
        PhoenixOdometryThread.getInstance(this.CANBusName).registerSignal(navX::getYaw);
    pitchPositionQueue =
        PhoenixOdometryThread.getInstance(this.CANBusName).registerSignal(navX::getPitch);
    rollPositionQueue =
        PhoenixOdometryThread.getInstance(this.CANBusName).registerSignal(navX::getRoll);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw());
    inputs.pitchPosition = Rotation2d.fromDegrees(-navX.getPitch());
    inputs.rollPosition = Rotation2d.fromDegrees(-navX.getRoll());

    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroY());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroX());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();

    yawPositionQueue.clear();
    pitchPositionQueue.clear();
    rollPositionQueue.clear();
  }
}
