package BobcatLib.Subsystems.Swerve.SimpleSwerve.Containers;

import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Commands.ControlledSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Commands.TeleopSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.Utility.LoadablePathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SwerveBase {
  /* Subsystems */
  public final OI s_Controls; // Interfaces with popular controllers and input devices
  public final OI s_ControlsSecondary; // Interfaces with popular controllers and input devices
  public SwerveDrive s_Swerve; // This is the
  // Swerve
  // Library
  // implementation.
  private LoggedDashboardChooser<Command> autoChooser; // Choose
  // an
  // Auto!
  private final List<LoadablePathPlannerAuto> autos;
  private final Field2d field;
  private final Alliance alliance;
  private final PIDConstants tranPidPathPlanner, rotPidPathPlanner;

  private final boolean isDual;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SwerveBase(
      OI driver_controller,
      List<LoadablePathPlannerAuto> autos,
      String robotName,
      boolean isSim,
      Alliance alliance,
      PIDConstants tranPidPathPlanner,
      PIDConstants rotPidPathPlanner) {
    this.isDual = false;
    this.s_Controls = driver_controller;
    this.s_ControlsSecondary = null;
    this.autos = autos;
    this.alliance = alliance;
    this.tranPidPathPlanner = tranPidPathPlanner;
    this.rotPidPathPlanner = rotPidPathPlanner;
    autoChooser = new LoggedDashboardChooser<>("Auto Routine"); // Choose an Auto!
    s_Swerve = new SwerveDrive(robotName, isSim, alliance);

    initComand();

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Configure AutoBuilder last
    configureAutos();

    // Configure the button bindings
    configureButtonBindings();
  }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SwerveBase(
      OI driver_controller,
      OI secondary_controller,
      List<LoadablePathPlannerAuto> autos,
      String robotName,
      boolean isSim,
      Alliance alliance,
      PIDConstants tranPidPathPlanner,
      PIDConstants rotPidPathPlanner) {

    this.isDual = true;
    this.s_Controls = driver_controller;
    this.s_ControlsSecondary = secondary_controller;
    this.autos = autos;
    this.alliance = alliance;
    this.tranPidPathPlanner = tranPidPathPlanner;
    this.rotPidPathPlanner = rotPidPathPlanner;
    autoChooser = new LoggedDashboardChooser<>("Auto Routine"); // Choose an Auto!
    s_Swerve = new SwerveDrive(robotName, isSim, alliance);

    initComand();

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Configure AutoBuilder last
    configureAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  public void initComand() {
    DoubleSupplier translation = () -> s_Controls.getLeftYValue();
    DoubleSupplier strafe = () -> s_Controls.getLeftXValue();
    DoubleSupplier rotate = () -> s_Controls.getRightXValue();
    if (isDual) {
      translation = () -> s_Controls.controller.getLeftYAxis();
      strafe = () -> s_Controls.controller.getLeftXAxis();
      rotate = () -> s_ControlsSecondary.controller.getLeftYAxis();
    } else {
      translation = () -> s_Controls.getLeftYValue();
      strafe = () -> s_Controls.getLeftXValue();
      rotate = () -> s_Controls.getRightXValue();
    }

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(s_Swerve, translation, strafe, rotate, s_Controls.controllerJson));
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
    // Sets Up PathPlanner with Swerve
    s_Swerve = s_Swerve.withPathPlanner(field, tranPid, rotPid);
    // Configure AutoBuilder last
    updateLoadedPaths(autos);
  }

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
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {
    Command zeroGyro = Commands.runOnce(s_Swerve::zeroHeading);
    Command setFieldCentric = Commands.runOnce(s_Swerve::setFieldCentric);
    s_Controls.zeroGyro.onTrue(zeroGyro);
    s_Controls.fieldCentric.toggleOnTrue(setFieldCentric);
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

  /**
   * Use this to pass the test command. Control pattern is forward, right , backwards, left, rotate
   * in place clockwise, rotate in place counterclowise, forward while rotating Clockwise, forward
   * while rotating counter clockwise
   *
   * @return the command to run in autonomous
   */
  public Command getTestCommand() {
    Command testSwerveForward =
        new ControlledSwerve(s_Swerve, 0.2, 0.0, 0.0, s_Controls.controllerJson).withTimeout(3);
    Command testSwerveRight =
        new ControlledSwerve(s_Swerve, 0.0, 0.2, 0.0, s_Controls.controllerJson).withTimeout(3);
    Command testSwerveBackwards =
        new ControlledSwerve(s_Swerve, -0.2, 0.0, 0.0, s_Controls.controllerJson).withTimeout(3);
    Command testSwerveLeft =
        new ControlledSwerve(s_Swerve, 0.0, -0.2, 0.0, s_Controls.controllerJson).withTimeout(3);
    Command testRIPCW =
        new ControlledSwerve(s_Swerve, 0.0, 0.0, 0.2, s_Controls.controllerJson).withTimeout(3);
    Command testRIPCCW =
        new ControlledSwerve(s_Swerve, 0.0, 0.0, -0.2, s_Controls.controllerJson).withTimeout(3);
    Command stopMotorsCmd = new InstantCommand(() -> s_Swerve.stopMotors());
    Command testCommand =
        testSwerveForward
            .andThen(testSwerveRight)
            .andThen(testSwerveBackwards)
            .andThen(testSwerveLeft)
            .andThen(testRIPCW)
            .andThen(testRIPCCW)
            .andThen(stopMotorsCmd);
    return testCommand;
  }

  public void setFieldCentric() {
    s_Swerve.setFieldCentric();
  }
}
