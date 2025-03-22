package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleJson;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public double offset = 0;
    public double velocity = 0.00;
    public double absAngle = 0;
    public double angle = 0;
  }

  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /**
   * sets the desired state of the module's angle and drive motors.
   *
   * @param desiredState
   * @param isOpenLoop
   */
  public default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}

  /** Reset the absolute encoder's position in rotations to the internal angle motor position. */
  public default void resetToAbsolute() {}

  /**
   * gets the current modules state in velocity and position given drive motor velocity in rotations
   * / sec given angle motor position in rotations
   *
   * @return swerve module state
   */
  public default SwerveModuleState getState() {
    return new SwerveModuleState() {};
  }

  /**
   * gets the position of the modules given motor position (drive ), and the angle motor's position
   *
   * @return swerve module position
   */
  public default SwerveModulePosition getPosition() {
    return new SwerveModulePosition() {};
  }

  /** Stops the motors properly. */
  public default void stopMotors() {}

  /**
   * @return motor Voltage of the given motor
   */
  public default double getVoltage() {
    return 0.0;
  }

  /* Sys ID component Module Stuff */
  public default void runCharachterization(double volts) {}

  /**
   * Gets the current position of the drive motor in meters
   *
   * @return drive motor position, in meters
   */
  public default double getPositionMeters() {
    return 0.0;
  }

  /**
   * Gets the current velocity of the drive motor in meters per second
   *
   * @return velocity, in meter per second
   */
  public default double getVelocityMetersPerSec() {
    return 0.0;
  }

  public default int getModuleNumber() {
    return 0;
  }

  public default SwerveModuleState getDesiredState() {
    return null;
  }

  public default ModuleJson getJsonConfig() {
    return new ModuleJson();
  }
}
