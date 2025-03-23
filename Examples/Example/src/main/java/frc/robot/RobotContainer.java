// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.config.PIDConstants;

import GalvonizedLib.Hardware.Controllers.ControllerWrapper;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.Drive;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.Containers.SwerveBase;
import GalvonizedLib.Subsystems.Swerve.Subsystems.drive.util.LoadablePathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer extends SwerveBase {
  public Drive drive;
  private LoggedDashboardChooser<Command> autoChooser; // Choose
  private final List<LoadablePathPlannerAuto> autos;
  private final PIDConstants tranPidPathPlanner, rotPidPathPlanner;
  private final ControllerWrapper driver_controller;

  public RobotContainer(Drive drive,
      ControllerWrapper driver_controller,
      List<LoadablePathPlannerAuto> autos,
      String robotName,
      boolean isSim,
      PIDConstants tranPidPathPlanner,
      PIDConstants rotPidPathPlanner) {
    super(drive, driver_controller, autos, robotName, isSim, tranPidPathPlanner, rotPidPathPlanner);
    this.drive = drive;
    this.driver_controller = driver_controller;
    this.autos = autos;
    this.tranPidPathPlanner = tranPidPathPlanner;
    this.rotPidPathPlanner = rotPidPathPlanner;
    configureContainerBindings();
  }

  private void configureContainerBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand(String name) {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return super.getAutonomousCommand(name);
  }

  public Field2d getField(){
    return super.getField();
  }

}
