package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleJson;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  public final int index;

  public SwerveModule(SwerveModuleIO io, int index) {
    this.io = io;
    this.index = index;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + Integer.toString(index), inputs);
  }

  /**
   * sets the desired state of the module's angle and drive motors.
   *
   * @param desiredState
   * @param isOpenLoop
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    io.setDesiredState(desiredState, isOpenLoop);
  }

  public int getModuleNumber() {
    return io.getModuleNumber();
  }

  /**
   * gets the current modules state in velocity and position given drive motor velocity in rotations
   * / sec given angle motor position in rotations
   *
   * @return swerve module state
   */
  public SwerveModuleState getState() {
    return io.getState();
  }

  public SwerveModuleState getDesiredState() {
    return io.getDesiredState();
  }

  /**
   * gets the position of the modules given motor position (drive ), and the angle motor's position
   *
   * @return swerve module position
   */
  public SwerveModulePosition getPosition() {
    return io.getPosition();
  }

  /** Reset the absolute encoder's position in rotations to the internal angle motor position. */
  public void resetToAbsolute() {
    io.resetToAbsolute();
  }

  /**
   * Gets the current position of the drive motor in meters
   *
   * @return drive motor position, in meters
   */
  public double getPositionMeters() {
    return io.getPositionMeters();
  }

  /* Sys ID component Module Stuff */
  public void runCharachterization(double volts) {
    io.runCharachterization(volts);
  }

  /**
   * Gets the current velocity of the drive motor in meters per second
   *
   * @return velocity, in meter per second
   */
  public double getVelocityMetersPerSec() {
    return io.getVelocityMetersPerSec();
  }

  /**
   * @return motor Voltage of the given motor
   */
  public double getVoltage() {
    return io.getVoltage();
  }

  /** Stops the motors properly. */
  public void stopMotors() {
    io.stopMotors();
  }

  public ModuleJson getJsonConfig() {
    return io.getJsonConfig();
  }
}
