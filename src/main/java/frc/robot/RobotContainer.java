// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public RobotContainer() {
    configureBindings();
    m_exampleSubsystem.initShuffleboard(LOG_LEVEL.OFF);
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
