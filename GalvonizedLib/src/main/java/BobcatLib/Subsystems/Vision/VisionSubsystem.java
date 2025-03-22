package BobcatLib.Subsystems.Vision;

import BobcatLib.Subsystems.Vision.Components.VisionIO;
import BobcatLib.Subsystems.Vision.Components.VisionLLIOInputsAutoLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Represents the Intake Subsystem in the robot. It interacts with the intake module to control the
 * intake mechanism, load configurations from a file, and periodically update inputs.
 */
public class VisionSubsystem extends SubsystemBase {
  public VisionIO io;
  public VisionLLIOInputsAutoLogged inputs = new VisionLLIOInputsAutoLogged();
  public final String name;
  /**
   * Constructor for the VisionSubsystem.
   *
   * @param name The name of the Vision subsystem.
   */
  public VisionSubsystem(String name, VisionIO io) {
    this.name = name;
    this.io = io;
  }

  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }
}
