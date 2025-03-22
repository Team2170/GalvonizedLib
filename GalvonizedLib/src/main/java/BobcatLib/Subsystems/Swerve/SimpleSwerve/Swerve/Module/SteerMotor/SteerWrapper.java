package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor;

/** Steer Wrapper */
public interface SteerWrapper {

  public default void updateOutputs() {}

  /*
   *
   * sets the angle of the motor by setting the internal pid.
   *
   * @param rotations
   */
  public default void setAngle(double rotations) {}

  /**
   * sets the position of the steer motor given the desired rotations.
   *
   * @param absolutePosition
   */
  public default void setPosition(double absolutePosition) {}

  /**
   * gets the position of the steer motor.
   *
   * @return position of the steer motor in rotations.
   */
  public default double getPosition() {
    return 0;
  }

  /** Stops the motors properly. */
  public default void stopMotor() {}

  public default void withSupplyCurrent() {}

  public default void withStatorCurrent() {}
}
