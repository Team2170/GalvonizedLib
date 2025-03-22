package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Drive Wrapper */
public interface DriveWrapper {

  public default void updateOutputs() {}

  /**
   * gets the velocity of the drive motor in rotations per second either using percentage output (
   * dutycycleout ) or velocity control triggered by the isOpenLoop parameter
   *
   * @param desiredState desired State
   * @param isOpenLoop open Loop
   */
  public default void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {}

  /**
   * gets the position of the drive motor in rotations
   *
   * @return position
   */
  public default double getPosition() {
    return 0;
  }

  /**
   * gets the velocity of the drive motor in rotations per second.
   *
   * @return velocity
   */
  public default double getVelocity() {
    return 0;
  }

  /**
   * Sets up the control mode for SYSID ONLY!
   *
   * @param volts
   */
  public default void setControl(double volts) {}

  /**
   * Gets motor Voltage of the given motor
   *
   * @return motorVoltage
   */
  public default double getMotorVoltage() {
    return 0;
  }

  /** Stops the motors properly. */
  public default void stopMotor() {}

  public default void withSupplyCurrent() {}

  public default void withStatorCurrent() {}
}
