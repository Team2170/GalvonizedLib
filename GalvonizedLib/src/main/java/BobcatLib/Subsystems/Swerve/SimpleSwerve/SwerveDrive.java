package BobcatLib.Subsystems.Swerve.SimpleSwerve;

import BobcatLib.Hardware.Gyros.BaseGyro;
import BobcatLib.Hardware.Gyros.GyroSim;
import BobcatLib.Hardware.Gyros.Pigeon2Gyro;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SwerveModule;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SwerveModuleReal;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SwerveModuleSim;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose.WpiPoseEstimator;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.BaseJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.SwerveIndicatorJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.SwerveJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.chassisLimitsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.driveCharacteristicsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.drivePIDJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.interfaces.AutomatedSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.interfaces.SysidCompatibleSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.StandardDeviations.OdometryStates;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.math.GeometryUtils;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase implements SysidCompatibleSwerve, AutomatedSwerve {
  public SwerveModule[] mSwerveMods;

  private final BaseGyro gyro;
  // Automated Interface Properties
  private Rotation2d autoAlignAngle = new Rotation2d();
  private Translation2d aimAssistTranslation = new Translation2d();
  public boolean isSim = false;
  private final PathConstraints pathfindingConstraints;
  /* Drivetrain Constants */
  public double trackWidth;
  public double wheelBase;
  public SwerveJson jsonSwerve = new SwerveJson();

  public PIDConstants pidTranslation;
  public PIDConstants pidRotation;
  public final WpiPoseEstimator swerveDrivePoseEstimator;
  private Alliance team;
  Matrix<N3, N1> visionStdDevs;
  Matrix<N3, N1> stateStdDevs;
  private String robotName;
  private final PIDController autoAlignPID;

  public boolean fieldCentric = false;
  /*
   * Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional
   * rectangular/square 4 module swerve
   */
  public static SwerveDriveKinematics swerveKinematics;

  public Matrix<N3, N1> trustautostdDev;
  public Matrix<N3, N1> trusttelestdDev;
  public Matrix<N3, N1> regautostdDev;
  public Matrix<N3, N1> regtelestdDev;

  public SwerveDrive(
      String robotName,
      boolean isSim,
      Alliance team,
      Matrix<N3, N1> visionStdDevs,
      Matrix<N3, N1> stateStdDevs) {
    this.team = team;
    this.isSim = isSim;
    this.visionStdDevs = visionStdDevs;
    this.stateStdDevs = stateStdDevs;
    this.robotName = robotName;

    /* Drivetrain Constants */
    loadConfigurationFromFile();
    trackWidth = Units.inchesToMeters(jsonSwerve.base.trackWidth);
    wheelBase = Units.inchesToMeters(jsonSwerve.base.wheelBase);
    double maxAngularAccel = jsonSwerve.moduleSpeedLimits.maxAngularAcceleration;
    double maxAngularVelocity = jsonSwerve.moduleSpeedLimits.maxAngularVelocity;
    pathfindingConstraints =
        new PathConstraints(
            jsonSwerve.moduleSpeedLimits.maxSpeed,
            jsonSwerve.moduleSpeedLimits.maxAccel,
            maxAngularVelocity,
            maxAngularAccel);
    swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    /* Setup Modules */
    if (isSim) {
      gyro = new BaseGyro("Swerve-Gyro", new GyroSim());
      mSwerveMods = new SwerveModule[] {};
    } else {
      gyro = new BaseGyro("Swerve-Gyro", new Pigeon2Gyro(robotName));
      mSwerveMods =
          new SwerveModule[] {
            new SwerveModule(new SwerveModuleReal(0, jsonSwerve.moduleSpeedLimits, robotName), 0),
            new SwerveModule(new SwerveModuleReal(1, jsonSwerve.moduleSpeedLimits, robotName), 1),
            new SwerveModule(new SwerveModuleReal(2, jsonSwerve.moduleSpeedLimits, robotName), 2),
            new SwerveModule(new SwerveModuleReal(3, jsonSwerve.moduleSpeedLimits, robotName), 3)
          };
    }
    Timer.delay(1);
    resetModulesToAbsolute();

    swerveDrivePoseEstimator =
        new WpiPoseEstimator(
            swerveKinematics,
            gyro.getYaw(),
            getModulePositions(),
            new Pose2d()); // x,y,heading in radians; Vision measurement std dev, higher=less
    // weight

    pidTranslation =
        new PIDConstants(
            jsonSwerve.translationPID.driveKP,
            jsonSwerve.translationPID.driveKI,
            jsonSwerve.translationPID.driveKD); // Translation
    pidRotation =
        new PIDConstants(
            jsonSwerve.rotationPID.driveKP,
            jsonSwerve.rotationPID.driveKI,
            jsonSwerve.rotationPID.driveKD); // Rotation
    autoAlignPID =
        new PIDController(
            jsonSwerve.autoAlignPID.driveKP,
            jsonSwerve.autoAlignPID.driveKI,
            jsonSwerve.autoAlignPID.driveKD);
  }

  public SwerveDrive(String robotName, boolean isSim, Alliance team) {
    this.team = team;
    this.isSim = isSim;
    this.visionStdDevs = null;
    this.stateStdDevs = null;
    this.robotName = robotName;

    /* Drivetrain Constants */
    loadConfigurationFromFile();
    trackWidth = Units.inchesToMeters(jsonSwerve.base.trackWidth);
    wheelBase = Units.inchesToMeters(jsonSwerve.base.wheelBase);
    double maxAngularAccel = jsonSwerve.moduleSpeedLimits.maxAngularAcceleration;
    double maxAngularVelocity = jsonSwerve.moduleSpeedLimits.maxAngularVelocity;
    pathfindingConstraints =
        new PathConstraints(
            jsonSwerve.moduleSpeedLimits.maxSpeed,
            jsonSwerve.moduleSpeedLimits.maxAccel,
            maxAngularVelocity,
            maxAngularAccel);
    swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    /* Setup Modules */
    if (isSim) {
      mSwerveMods =
          new SwerveModule[] {
            new SwerveModule(new SwerveModuleSim(0, jsonSwerve.moduleSpeedLimits, robotName), 0),
            new SwerveModule(new SwerveModuleSim(1, jsonSwerve.moduleSpeedLimits, robotName), 1),
            new SwerveModule(new SwerveModuleSim(2, jsonSwerve.moduleSpeedLimits, robotName), 2),
            new SwerveModule(new SwerveModuleSim(3, jsonSwerve.moduleSpeedLimits, robotName), 3)
          };
      gyro = new BaseGyro("Swerve-Gyro", new GyroSim());
    } else {
      mSwerveMods =
          new SwerveModule[] {
            new SwerveModule(new SwerveModuleReal(0, jsonSwerve.moduleSpeedLimits, robotName), 0),
            new SwerveModule(new SwerveModuleReal(1, jsonSwerve.moduleSpeedLimits, robotName), 1),
            new SwerveModule(new SwerveModuleReal(2, jsonSwerve.moduleSpeedLimits, robotName), 2),
            new SwerveModule(new SwerveModuleReal(3, jsonSwerve.moduleSpeedLimits, robotName), 3)
          };

      gyro = new BaseGyro("Swerve-Gyro", new Pigeon2Gyro(robotName));
    }
    Timer.delay(1);
    resetModulesToAbsolute();

    swerveDrivePoseEstimator =
        new WpiPoseEstimator(
            swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(
                new Translation2d(0, 0),
                Rotation2d.fromDegrees(
                    0))); // x,y,heading in radians; Vision measurement std dev, higher=less weight

    pidTranslation =
        new PIDConstants(
            jsonSwerve.translationPID.driveKP,
            jsonSwerve.translationPID.driveKI,
            jsonSwerve.translationPID.driveKD); // Translation
    pidRotation =
        new PIDConstants(
            jsonSwerve.rotationPID.driveKP,
            jsonSwerve.rotationPID.driveKI,
            jsonSwerve.rotationPID.driveKD); // Rotation

    autoAlignPID =
        new PIDController(
            jsonSwerve.autoAlignPID.driveKP,
            jsonSwerve.autoAlignPID.driveKI,
            jsonSwerve.autoAlignPID.driveKD);
  }

  public SwerveDrive withPathPlanner(
      Field2d field, PIDConstants translationPid, PIDConstants rotationPid) {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      try {
        throw new Exception("Could not configure Robot Config in PathPlanner!");
      } catch (Exception e1) {
        // TODO Auto-generated catch block
        e1.printStackTrace();
      }
      return this;
    }

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.setRobotPose(pose);
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
        });

    AutoBuilder.configure(
        this::getPose, // Supplier of current robot pose
        (pose) -> setPose(pose), // Consumer for seeding pose against auto
        () -> getChassisSpeeds(), // Supplier of current robot speeds
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        (speeds, feedforwards) -> drive(speeds),
        new PPHolonomicDriveController(
            // PID constants for translation
            translationPid.asPathPlanner(),
            // PID constants for rotation
            rotationPid.asPathPlanner()),
        config,
        // Assume the path needs to be flipped for Red vs Blue, this is normally the
        // case
        () -> false,
        this // Subsystem for requirements
        );
    Pathfinding.setPathfinder(new LocalADStar());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          final Pose2d[] trajectory = activePath.toArray(new Pose2d[0]);
          Logger.recordOutput("Odometry/Trajectory", trajectory);
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));
    return this;
  }

  public SwerveJson loadConfigurationFromFile() {
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve/" + robotName + "/");
    assert directory.exists();
    assert new File(directory, "swerve.json").exists();
    File swerveFile = new File(directory, "swerve.json");
    assert swerveFile.exists();
    jsonSwerve = new SwerveJson();
    try {
      jsonSwerve = new ObjectMapper().readValue(swerveFile, SwerveJson.class);
      SmartDashboard.putNumber(
          "Swerve ChassisLimits maxSpeed", jsonSwerve.chassisSpeedLimits.maxSpeed);
      SmartDashboard.putNumber(
          "Swerve ModuleLimits maxSpeed", jsonSwerve.moduleSpeedLimits.maxSpeed);
      SmartDashboard.putString(
          "Swerve Base", jsonSwerve.base.trackWidth + " x " + jsonSwerve.base.wheelBase);
      SmartDashboard.putNumber("Swerve Indicator Id", jsonSwerve.indicator.id);
      SmartDashboard.putNumber("Swerve Rotation PID", jsonSwerve.rotationPID.driveKP);
      SmartDashboard.putNumber("Swerve Transalation PID", jsonSwerve.translationPID.driveKP);

    } catch (IOException e) {
      jsonSwerve.base = new BaseJson();
      jsonSwerve.chassisSpeedLimits = new chassisLimitsJson();
      jsonSwerve.driveCharacteristics = new driveCharacteristicsJson();
      jsonSwerve.moduleSpeedLimits = new ModuleLimitsJson();
      jsonSwerve.rotationPID = new drivePIDJson();
      jsonSwerve.translationPID = new drivePIDJson();
      jsonSwerve.indicator = new SwerveIndicatorJson();
    }
    return jsonSwerve;
  }

  public Command driveAsCommand(Translation2d translation) {
    Rotation2d currentHeading = new Rotation2d();
    Pose2d currentPose = new Pose2d();
    Command driveCmd =
        new InstantCommand(() -> drive(translation, 0, false, currentHeading, currentPose));
    return driveCmd;
  }

  /**
   * Make the swerve drive move
   *
   * @param targetSpeeds the desired chassis speeds
   */
  public void drive(ChassisSpeeds targetSpeeds) {
    SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(targetSpeeds);
    double maxSpeed = jsonSwerve.moduleSpeedLimits.maxSpeed;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], true);
    }
  }

  /**
   * drive the swerve with 1st or 2nd order.
   *
   * @param translation
   * @param rotation
   * @param isOpenLoop
   * @param currentHeading
   * @param currentPose
   */
  public void drive(
      Translation2d translation,
      double rotation,
      boolean isOpenLoop,
      Rotation2d currentHeading,
      Pose2d currentPose) {
    Logger.recordOutput("Swerve/FieldCentric", fieldCentric);
    currentHeading = getGyroYaw();
    currentPose = getPose();
    if (Constants.SwerveConstants.firstOrderDriving) {
      drive1stOrder(translation, rotation, isOpenLoop, currentHeading, currentPose);
    } else {
      drive2ndOrder(translation, rotation, isOpenLoop, currentHeading, currentPose);
    }
  }

  /**
   * drive the swerve with 1st or 2nd order. But with an auto align target rotation.
   *
   * @param translation
   * @param isOpenLoop
   * @param currentHeading rotation2d of the robots current heading
   * @param currentPose
   * @param TargetYaw in degrees
   */
  public void driveWithAutoAlign(
      Translation2d translation,
      boolean isOpenLoop,
      Rotation2d currentHeading,
      Pose2d currentPose,
      double TargetYaw) {
    currentHeading = getHeading();
    currentPose = getPose();

    double rotation = autoAlignPID.calculate(getGyroYaw().getDegrees(), TargetYaw);

    if (Constants.SwerveConstants.firstOrderDriving) {
      drive1stOrder(translation, rotation, isOpenLoop, currentHeading, currentPose);
    } else {
      drive2ndOrder(translation, rotation, isOpenLoop, currentHeading, currentPose);
    }
  }

  /**
   * Drive using second order
   *
   * @param translation
   * @param rotation
   * @param isOpenLoop
   * @param currentHeading
   * @param currentPose
   */
  public void drive1stOrder(
      Translation2d translation,
      double rotation,
      boolean isOpenLoop,
      Rotation2d currentHeading,
      Pose2d currentPose) {
    SwerveModuleState[] swerveModuleStates;

    if (fieldCentric) {
      swerveModuleStates =
          swerveKinematics.toSwerveModuleStates(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  translation.getX(), translation.getY(), rotation, currentHeading));
    } else {
      swerveModuleStates =
          swerveKinematics.toSwerveModuleStates(
              new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    }
    double maxSpeed = jsonSwerve.moduleSpeedLimits.maxSpeed;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    applyModuleStates(swerveModuleStates, isOpenLoop);
  }

  /**
   * Drive using second order
   *
   * @param translation
   * @param rotation
   * @param isOpenLoop
   * @param currentHeading
   * @param currentPose
   */
  public void drive2ndOrder(
      Translation2d translation,
      double rotation,
      boolean isOpenLoop,
      Rotation2d currentHeading,
      Pose2d currentPose) {
    ChassisSpeeds desiredChassisSpeeds;
    if (fieldCentric) {
      desiredChassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(),
              translation.getY(),
              rotation,
              currentPose
                  .getRotation()
                  .plus(
                      Rotation2d.fromDegrees(team.get() == DriverStation.Alliance.Red ? 180 : 0)));
    } else {
      desiredChassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
    SwerveModuleState[] swerveModuleStates =
        swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    double maxSpeed = jsonSwerve.moduleSpeedLimits.maxSpeed;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    applyModuleStates(swerveModuleStates, isOpenLoop);
  }

  /**
   * Applies the current desired states to the modules.
   *
   * @param swerveModuleStates
   * @param isOpenLoop
   */
  private void applyModuleStates(SwerveModuleState[] swerveModuleStates, boolean isOpenLoop) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
    }
  }

  /**
   * sets the swerve's desired states
   *
   * @param desiredStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    double maxSpeed = jsonSwerve.moduleSpeedLimits.maxSpeed;
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
    applyModuleStates(desiredStates, false);
  }

  /**
   * Gets the module states
   *
   * @return
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.getModuleNumber()] = mod.getState();
    }
    return states;
  }

  /**
   * Gets the desired module states
   *
   * @return
   */
  public SwerveModuleState[] getDesiredModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.getModuleNumber()] = mod.getDesiredState();
    }
    return states;
  }

  /**
   * Gets the module position
   *
   * @return
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.getModuleNumber()] = mod.getPosition();
    }
    return positions;
  }

  /**
   * @return
   */
  public Rotation2d getGyroYaw() {
    return gyro.getYaw();
  }

  /**
   * Read the modules absolute encoder and set the internal steer motors too the absoluate. angle
   * set in units of rotations.
   */
  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Correct drift dynamically
   *
   * @param originalSpeeds
   * @return
   */
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  /**
   * Gets the speeds from the modules as defined as chassis.
   *
   * @return ChassisSpeeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    // Simple Second Order Kinematics
    ChassisSpeeds internalChassisSpeeds = swerveKinematics.toChassisSpeeds(getModuleStates());
    return correctForDynamics(internalChassisSpeeds);
  }

  @Override
  public void periodic() {
    updateGyroInSim();
    gyro.periodic();
    for (SwerveModule mod : mSwerveMods) {
      mod.periodic();
    }

    // swerveDrivePoseEstimator.setStateStdDevs(new state std devs here);
    handleOdometry();
    // swerveDrivePoseEstimator.addVisionMeasurement();

    Logger.recordOutput("Swerve/Rotation", getGyroYaw());
    Logger.recordOutput("Swerve/DesiredModuleStates", getDesiredModuleStates());
    Logger.recordOutput("Swerve/ModuleStates", getModuleStates());
    Logger.recordOutput("Swerve/Pose", getPose());
  }

  public void handleOdometry() {
    OdometryStates state;
    // determine how much to trust odometry based on acceleratio
    if (gyro.getAccel() > 5) {
      state = OdometryStates.THROWOUT;
      swerveDrivePoseEstimator.updateWithTime(
          Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
    } else if (gyro.getAccel() > 4) {
      state = OdometryStates.DISTRUST;
      swerveDrivePoseEstimator.updateWithTime(
          Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
    } else {
      state = OdometryStates.TRUST;
      swerveDrivePoseEstimator.updateWithTime(
          Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
    }
  }

  /** Checks if in sim. */
  private void updateGyroInSim() {
    if (!isSim) {
      return;
    }

    double angle = getChassisSpeeds().omegaRadiansPerSecond * gyro.getTimeDiff();
    gyro.setYaw(angle);
  }

  /** Stops the motors properly. */
  public void stopMotors() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stopMotors();
    }
  }

  /**
   * Add vision measurement to swerve drivetrain "this should be moved into the vision specific
   * stuff." - AO
   *
   * @param pose
   */
  public Command driveToPose(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, pathfindingConstraints);
  }

  /* Automated Swerve Classes */
  @Override
  public Rotation2d autoAlignAngle() {
    return autoAlignAngle;
  }

  @Override
  public void setAutoAlignAngle(Rotation2d angle) {
    autoAlignAngle = angle;
  }

  @Override
  public Translation2d aimAssistTranslation() {
    return aimAssistTranslation;
  }

  @Override
  public void setAimAssistTranslation(Translation2d translation) {
    aimAssistTranslation = translation;
  }
  /* end aim assist stuff */

  /* sysid stuff */

  /** set all modules to supplied voltage */
  @Override
  public void sysidVoltage(Measure<VoltageUnit> volts) {
    for (SwerveModule mod : mSwerveMods) {
      mod.runCharachterization(volts.magnitude());
    }
  }

  /**
   * volts
   *
   * <p>index of module number starts at 0
   */
  @Override
  public double getModuleVoltage(int moduleNumber) {
    return mSwerveMods[moduleNumber].getVoltage();
  }

  /** meters */
  @Override
  public double getModuleDistance(int moduleNumber) {
    return mSwerveMods[moduleNumber].getPositionMeters();
  }

  /** meters/sec */
  @Override
  public double getModuleSpeed(int moduleNumber) {
    return mSwerveMods[moduleNumber].getVelocityMetersPerSec();
  }

  /* end sysid stuff */

  /* Odometry / Pose items */

  /**
   * Fetch the latest odometry heading, should be trusted over {@link SwerveDrive#getGyroYaw()}.
   *
   * @return {@link Rotation2d} of the robot heading.
   */
  public Rotation2d getHeading() {
    return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Gets the pose from the swerve pose.
   *
   * @return
   */
  public Pose2d getPose() {
    Pose2d poseEstimation = swerveDrivePoseEstimator.getEstimatedPosition();
    return poseEstimation;
  }

  /**
   * Sets the pose from the swerve system using gyro.
   *
   * @param pose
   */
  public void setPose(Pose2d pose) {
    swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroYaw()));
  }

  /** Zeros the heading of the swerve based on the gyro */
  public void zeroHeading() {
    swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), getPose());
    swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroYaw()));
  }

  public void setFieldCentric() {
    fieldCentric = fieldCentric ? false : true;
  }

  public BaseGyro getBaseGyro() {
    return gyro;
  }
}
