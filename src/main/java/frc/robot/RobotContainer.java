// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.RobotStateSubsystem.RobotStateSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.laser.LaserSubsystem;
import frc.robot.subsystems.laser.LaserSubsystemIOFX;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystemIOFX;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem;
  private Swerve swerve;
  private final LaserSubsystem laserSubsystem;
  private final LaserSubsystemIOFX laserIO;
  private final RobotStateSubsystem robotStateSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final TurretSubsystemIOFX turretIO;
  private final VisionSubsystem visionSubsystem;

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem(swerve);
    laserIO = new LaserSubsystemIOFX();
    laserSubsystem = new LaserSubsystem(laserIO);
    turretIO = new TurretSubsystemIOFX();
    turretSubsystem = new TurretSubsystem(turretIO);
    visionSubsystem = new VisionSubsystem(turretSubsystem, driveSubsystem);
    robotStateSubsystem = new RobotStateSubsystem(turretSubsystem, laserSubsystem, driveSubsystem);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
