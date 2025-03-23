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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> rollPositionQueue;
  private final Queue<Double> pitchPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<AngularVelocity> rollVelocity;
  private final StatusSignal<AngularVelocity> pitchVelocity;
  private final String CANBusName;

  public GyroIOPigeon2(int Pigeon2Id, String CANBusName) {
    pigeon = new Pigeon2(Pigeon2Id, CANBusName);
    this.CANBusName = CANBusName;
    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    rollVelocity = pigeon.getAngularVelocityXWorld();
    pitchVelocity = pigeon.getAngularVelocityYWorld();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    pitch.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    roll.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pitchVelocity.setUpdateFrequency(50.0);
    rollVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance(this.CANBusName).makeTimestampQueue();
    yawPositionQueue =
        PhoenixOdometryThread.getInstance(this.CANBusName).registerSignal(pigeon.getYaw());
    pitchPositionQueue =
        PhoenixOdometryThread.getInstance(this.CANBusName).registerSignal(pigeon.getPitch());
    rollPositionQueue =
        PhoenixOdometryThread.getInstance(this.CANBusName).registerSignal(pigeon.getRoll());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
    inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();

    yawPositionQueue.clear();
    pitchPositionQueue.clear();
    rollPositionQueue.clear();
  }
}
