package GalvonizedLib.Subsystems.Swerve.Subsystems.drive.Containers;

import GalvonizedLib.Hardware.Controllers.ControllerWrapper;
import GalvonizedLib.Subsystems.Swerve.Commands.CharacterizationCommands;
import GalvonizedLib.Subsystems.Swerve.Commands.DriveCommands;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.Drive;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.util.LoadablePathPlannerAuto;
import GalvonizedLib.Subsystems.Swerve.Subsystems.vision.Vision;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SwerveBase {
  public Drive drive;
  private LoggedDashboardChooser<Command> autoChooser; // Choose
  private final List<LoadablePathPlannerAuto> autos;
  private final PIDConstants tranPidPathPlanner, rotPidPathPlanner;
  private final Field2d field;
  private final ControllerWrapper driver_controller;
  private List<Vision> cameras;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SwerveBase(
      Drive drive,
      ControllerWrapper driver_controller,
      List<LoadablePathPlannerAuto> autos,
      String robotName,
      boolean isSim,
      PIDConstants tranPidPathPlanner,
      PIDConstants rotPidPathPlanner) {
    this.drive = drive;
    this.driver_controller = driver_controller;
    this.autos = autos;
    this.tranPidPathPlanner = tranPidPathPlanner;
    this.rotPidPathPlanner = rotPidPathPlanner;
    autoChooser = new LoggedDashboardChooser<>("Auto Routine"); // Choose an Auto!

    initComand();

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Configure AutoBuilder last
    configureAutos();

    // Configure the button bindings
    configureButtonBindings();

    cameras = new ArrayList<Vision>() {};
  }

  public SwerveBase withVision(List<Vision> cameras) {
    this.cameras = cameras;
    return this;
  }

  public void initComand() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.JoystickLimitedDrive(
            drive,
            () -> driver_controller.getLeftYAxis(),
            () -> driver_controller.getLeftXAxis(),
            () -> driver_controller.getLeftTriggerValue(),
            () -> driver_controller.getRightTriggerValue(),
            () -> -driver_controller.getRightXAxis(),
            driver_controller.getAorCross()));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        CharacterizationCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        CharacterizationCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public boolean autoChooserInitialized() {
    return autoChooser.get() != null;
  }

  public LoggedDashboardChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  /** this should only be called once DS and FMS are attached */
  public void configureAutos() {
    // PID constants for translation
    PIDConstants tranPid = tranPidPathPlanner;
    // PID constants for rotation
    PIDConstants rotPid = rotPidPathPlanner;
    // Configure AutoBuilder last
    updateLoadedPaths(autos);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public void updateLoadedPaths(List<LoadablePathPlannerAuto> loadedAutos) {
    for (LoadablePathPlannerAuto auto : autos) {
      if (auto.isDefault()) {
        autoChooser.addDefaultOption(auto.getName(), auto.getCommand());
      } else {
        autoChooser.addOption(auto.getName(), auto.getCommand());
      }
    }
  }

  /**
   * Use this to pass the autonomous command.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String name) {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    LoadablePathPlannerAuto t =
        autos.stream().filter(target -> target.getName() == name).findFirst().orElse(null);
    return t.getCommand();
  }

  public void periodic() {
    for (Vision camera : cameras) {
      camera.periodic();
    }
  }

  public Field2d getField() {
    return field;
  }
}
