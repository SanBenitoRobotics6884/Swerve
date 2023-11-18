// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FieldDrive;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.SQUARED_INPUTS;

public class RobotContainer {
  private CommandXboxController m_controller = new CommandXboxController(1);

  private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private DefaultDrive m_defaultDrive = new DefaultDrive(
      m_swerveSubsystem,
      () -> input(m_controller.getLeftY(), SQUARED_INPUTS),
      () -> input(m_controller.getLeftX(), SQUARED_INPUTS),
      () -> input(m_controller.getRightX(), SQUARED_INPUTS)); 

  private FieldDrive m_fieldDrive = new FieldDrive(
      m_swerveSubsystem,
      () -> input(m_controller.getLeftY(), SQUARED_INPUTS),
      () -> input(m_controller.getLeftX(), SQUARED_INPUTS),
      () -> input(m_controller.getRightX(), SQUARED_INPUTS)); 

  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(m_defaultDrive);

    configureBindings();
  }

  private void configureBindings() {
    m_controller.a().toggleOnTrue(m_fieldDrive);
    m_controller.y().onTrue(Commands.runOnce(m_swerveSubsystem::zeroYaw));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private double input(double input, boolean squared) {
    return squared ? (input > 0 ? 1 : -1) * Math.pow(input, 2) : input;
  }
}
