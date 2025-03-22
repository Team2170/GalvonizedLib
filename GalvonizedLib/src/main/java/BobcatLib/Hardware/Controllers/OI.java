package BobcatLib.Hardware.Controllers;

import BobcatLib.Hardware.Controllers.parser.ControllerJson;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import java.io.IOException;

/**
 * The Operator Interface (OI) class manages the controller inputs and button mappings for the
 * robot's driver. It supports various controller types and their configurations.
 */
public class OI {
  /** The driver joystick. */
  public Joystick joystick;

  /** Driver Buttons */
  public Trigger fieldCentric;

  /** Driver Zero Gyros */
  public Trigger zeroGyro;

  public Trigger xMode;

  /* controller configuration */
  public ControllerJson controllerJson;

  /** The wrapper for the driver controller, supporting different controller types. */
  public ControllerWrapper controller;

  private String robotName;

  public String type;

  /**
   * Constructs the Operator Interface (OI) with the specified driver port and configuration.
   * Initializes controller inputs and button mappings.
   */
  public OI(String robotName) {
    this.robotName = robotName;
    loadConfigurationFromFile();
    int driverPort = controllerJson.single.id;
    type = controllerJson.single.type;
    joystick = new Joystick(driverPort);
    controller = init(type, driverPort);
  }

  public ControllerWrapper init(String type, int driverPort) {
    ControllerWrapper tmp;
    switch (type) {
      case "xbox":
        tmp = new XboxControllerWrapper(driverPort);
        break;
      case "ps4":
        tmp = new PS4ControllerWrapper(driverPort);
        break;
      case "ps5":
        tmp = new PS5ControllerWrapper(driverPort);
        break;
      case "ruffy":
        tmp = new Ruffy(driverPort);
        break;
      case "logitech":
        tmp = new Logitech(driverPort);
        break;
      case "eightbitdo":
        tmp = new EightBitDo(driverPort);
        break;
      default:
        tmp = new XboxControllerWrapper(driverPort);
        break;
    }
    return tmp;
  }

  public OI withXModeButton() {
    switch (type) {
      case "xbox":
        xMode = controller.getXorSquare();
        break;
      case "ps4":
        xMode = controller.getXorSquare();
        break;
      case "ps5":
        xMode = controller.getXorSquare();
        break;
      case "ruffy":
        xMode = controller.getTopButton();
        break;
      case "logitech":
        xMode = controller.getXorSquare();
        break;
      case "eightbitdo":
        xMode = controller.getXorSquare();
        break;
      default:
        xMode = controller.getXorSquare();
        break;
    }
    return this;
  }

  public OI withCentricButton() {
    switch (type) {
      case "xbox":
        fieldCentric = controller.getBorCircle();
        break;
      case "ps4":
        fieldCentric = controller.getBorCircle();
        break;
      case "ps5":
        fieldCentric = controller.getBorCircle();
        break;
      case "ruffy":
        fieldCentric = controller.getTopButton();
        break;
      case "logitech":
        fieldCentric = controller.getBorCircle();
        break;
      case "eightbitdo":
        fieldCentric = controller.getBorCircle();
        break;
      default:
        fieldCentric = controller.getBorCircle();
        break;
    }

    return this;
  }

  public OI withZeroButton() {
    switch (type) {
      case "xbox":
        fieldCentric = controller.getAorCross();
        break;
      case "ps4":
        fieldCentric = controller.getAorCross();
        break;
      case "ps5":
        fieldCentric = controller.getAorCross();
        break;
      case "ruffy":
        fieldCentric = controller.getAorCross();
        break;
      case "logitech":
        fieldCentric = controller.getAorCross();
        break;
      case "eightbitdo":
        fieldCentric = controller.getAorCross();
        break;
      default:
        fieldCentric = controller.getAorCross();
        break;
    }

    return this;
  }

  /**
   * Loads Configuration From Deployed File
   *
   * @return ControllerJson
   */
  public ControllerJson loadConfigurationFromFile() {
    String name = "oi.json";
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve/" + robotName + "/");
    assert new File(directory, name).exists();
    File moduleFile = new File(directory, name);
    assert moduleFile.exists();
    controllerJson = new ControllerJson();
    try {
      controllerJson = new ObjectMapper().readValue(moduleFile, ControllerJson.class);
    } catch (IOException e) {
      e.printStackTrace();
    }
    return controllerJson;
  }

  /**
   * Gets Driver LeftY Axis
   *
   * @return double
   */
  public double getLeftYValue() {
    return controller.getLeftYAxis();
  }

  /**
   * Gets Driver LeftX Axis
   *
   * @return double
   */
  public double getLeftXValue() {
    return controller.getLeftXAxis();
  }

  /**
   * Gets Driver RightX Axis
   *
   * @return double
   */
  public double getRightXValue() {
    return controller.getRightXAxis();
  }

  /**
   * Gets Driver RightY Axis
   *
   * @return double
   */
  public double getRightYValue() {
    return controller.getRightYAxis();
  }
}
