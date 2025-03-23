// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

import GalvonizedLib.GalvonizedLibCoreRobot;
import GalvonizedLib.Hardware.Controllers.XboxControllerWrapper;
import GalvonizedLib.Subsystems.Swerve.Constants.BuildConstants;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.Drive;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.GyroIO;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.GyroIOPigeon2;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.ModuleIO;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.ModuleIOSim;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.ModuleIOTalonFX;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.util.LoadablePathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants;
import frc.robot.generated.TunerConstants;

public class Robot extends GalvonizedLibCoreRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Drive drive;

  public Robot() {
    super(isSimulation());

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Check for valid swerve config
    var modules = new SwerveModuleConstants[] {
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight
    };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(TunerConstants.kPigeonId, TunerConstants.kCANBus.getName()),
            new ModuleIOTalonFX(TunerConstants.FrontLeft, TunerConstants.kCANBus.getName()),
            TunerConstants.FrontLeft, new ModuleIOTalonFX(TunerConstants.FrontRight, TunerConstants.kCANBus.getName()),
            TunerConstants.FrontRight, new ModuleIOTalonFX(TunerConstants.BackLeft, TunerConstants.kCANBus.getName()),
            TunerConstants.BackLeft, new ModuleIOTalonFX(TunerConstants.BackRight, TunerConstants.kCANBus.getName()),
            TunerConstants.BackRight, TunerConstants.kCANBus.getName(),
            TunerConstants.kSpeedAt12Volts, Constants.currentMode,
            TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos,
            TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos,
            TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos,
            TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIOPigeon2(TunerConstants.kPigeonId, TunerConstants.kCANBus.getName()),
            new ModuleIOSim(TunerConstants.FrontLeft),
            TunerConstants.FrontLeft, new ModuleIOSim(TunerConstants.FrontRight),
            TunerConstants.FrontRight, new ModuleIOSim(TunerConstants.BackLeft),
            TunerConstants.BackLeft, new ModuleIOSim(TunerConstants.BackRight),
            TunerConstants.BackRight, TunerConstants.kCANBus.getName(),
            TunerConstants.kSpeedAt12Volts, Constants.currentMode,
            TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos,
            TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos,
            TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos,
            TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            TunerConstants.FrontLeft, new ModuleIO() {
            },
            TunerConstants.FrontRight, new ModuleIO() {
            },
            TunerConstants.BackLeft, new ModuleIO() {
            },
            TunerConstants.BackRight, TunerConstants.kCANBus.getName(),
            TunerConstants.kSpeedAt12Volts, Constants.currentMode,
            TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos,
            TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos,
            TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos,
            TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos);
        break;
    }

    List<LoadablePathPlannerAuto> loadableAutos = new ArrayList<LoadablePathPlannerAuto>();
    loadableAutos.add(new LoadablePathPlannerAuto("Do Nothing", Commands.none(), true));
    String robotName = "2025_Robot";
    boolean isSim = false;
    PIDConstants tranPidPathPlanner = new PIDConstants(10, 0, 0);
    PIDConstants rotPidPathPlanner = new PIDConstants(5, 0, 0);

    m_robotContainer = new RobotContainer(drive, new XboxControllerWrapper(0),loadableAutos,robotName,isSim,
        tranPidPathPlanner,rotPidPathPlanner);
    drive.withPathPlanner(m_robotContainer.getField(), tranPidPathPlanner, rotPidPathPlanner);

    loadableAutos.add(new LoadablePathPlannerAuto("Base", new PathPlannerAuto("Base").withName("Base"), false));

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    String name = m_robotContainer.getAutoChooser().get().getName();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand(name);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
