package BobcatLib.Subsystems.Intakes;

import BobcatLib.Subsystems.Intakes.Modules.IntakeModuleIO;
import BobcatLib.Subsystems.Intakes.Modules.IntakeModuleIO.IntakeIOInputs;
import BobcatLib.Subsystems.Intakes.Parser.IntakeJson;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

/**
 * Represents the Intake Subsystem in the robot. It interacts with the intake module to control the
 * intake mechanism, load configurations from a file, and periodically update inputs.
 */
public class IntakeSubsystem extends SubsystemBase {
  private IntakeModuleIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private String name = "";
  private IntakeJson intakeJson;

  /**
   * Constructor for the IntakeSubsystem.
   *
   * @param name The name of the intake subsystem.
   * @param io The input/output interface for the intake module.
   */
  public IntakeSubsystem(String name, IntakeModuleIO io) {
    this.name = name;
    this.io = io;
  }

  /**
   * Loads the configuration for the intake subsystem from a JSON file located in the robot's deploy
   * directory. The file is expected to be in the "configs/Intake" folder.
   */
  public void loadConfigurationFromFile() {
    // Get the deploy directory where the configuration file is located
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();

    // Define the directory path and the intake JSON file path
    File directory = new File(deployDirectory, "configs/Intake");
    assert new File(directory, "intake.json").exists();
    File intakeFile = new File(directory, "intake.json");
    assert intakeFile.exists();

    // Parse the intake configuration JSON file
    intakeJson = new IntakeJson();
    try {
      intakeJson = new ObjectMapper().readValue(intakeFile, IntakeJson.class);
    } catch (IOException e) {
      // Handle the case where reading the file fails (you may want to log or throw an exception)
    }

    // Load the configuration into the IO interface
    io.loadConfigurationFromFile(intakeJson);
  }

  /**
   * Periodically updates the input values for the intake subsystem and logs them. This method is
   * typically called every robot cycle.
   */
  public void periodic() {
    io.updateInputs(inputs); // Update the intake subsystem inputs
  }

  /** Stops the intake subsystem by halting the IO operations. */
  public void stop() {
    io.stop(); // Stop the intake module IO
  }
}
