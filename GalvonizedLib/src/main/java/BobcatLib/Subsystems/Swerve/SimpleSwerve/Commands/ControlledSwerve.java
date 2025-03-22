package BobcatLib.Subsystems.Swerve.SimpleSwerve.Commands;

import BobcatLib.Hardware.Controllers.Axis;
import BobcatLib.Hardware.Controllers.parser.ControllerJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command class for controlling a swerve drive system with inputs provided by external suppliers.
 * This class executes periodic commands to manage translation, strafe, and rotation, and supports
 * robot-centric control.
 */
public class ControlledSwerve extends Command {

  /** Swerve drive subsystem. */
  private SwerveDrive s_Swerve;

  /** Supplier for translation control. */
  private Double translationSup;

  /** Supplier for strafe control. */
  private Double strafeSup;

  /** Supplier for rotation control. */
  private Double rotationSup;

  /** Flag for enabling robot-centric control. */
  private boolean fieldCentricSup;

  /** Configuration details parsed from a JSON controller configuration. */
  private ControllerJson controllerJson;

  /**
   * Constructor for the ControlledSwerve class.
   *
   * @param s_Swerve the swerve drive subsystem to control
   * @param translationSup supplier for translation (forward/backward) control
   * @param strafeSup supplier for strafing (side-to-side) control
   * @param rotationSup supplier for rotational control
   * @param controllerJson parsed configuration details for driver input and limits
   */
  public ControlledSwerve(
      SwerveDrive s_Swerve,
      Double translationSup,
      Double strafeSup,
      Double rotationSup,
      ControllerJson controllerJson) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.controllerJson = controllerJson;
  }

  /**
   * Periodic execution method called every 20ms during command execution. Calculates drive
   * parameters based on the input suppliers and applies them to the swerve drive system.
   */
  @Override
  public void execute() {
    // Apply deadband to control inputs
    Axis translation;
    Axis strafe;
    Axis rotation;

    if (controllerJson.isDual) {
      translation = new Axis(translationSup, controllerJson.single.deadband);
      strafe = new Axis(strafeSup, controllerJson.single.deadband);
      rotation = new Axis(rotationSup, controllerJson.split_one.deadband);
    } else {
      translation = new Axis(translationSup, controllerJson.single.deadband);
      strafe = new Axis(strafeSup, controllerJson.single.deadband);
      rotation = new Axis(rotationSup, controllerJson.single.deadband);
    }
    // Retrieve drive limits from the swerve configuration
    double maxSpeed = s_Swerve.jsonSwerve.moduleSpeedLimits.maxSpeed;
    double maxAngularVelocity = s_Swerve.jsonSwerve.moduleSpeedLimits.maxAngularVelocity;

    // Execute swerve drive command with calculated parameters
    s_Swerve.drive(
        new Translation2d(translation.getDeadband(), strafe.getDeadband()).times(maxSpeed),
        rotation.getDeadband() * maxAngularVelocity,
        true,
        s_Swerve.getHeading(),
        s_Swerve.getPose());
  }
}
