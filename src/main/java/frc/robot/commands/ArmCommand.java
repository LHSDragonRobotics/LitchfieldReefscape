// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.AbstractController;
import frc.robot.subsystems.BasicController;
import frc.robot.subsystems.BasicFlexController;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  protected final BasicFlexController m_subsystem;
  private double commandPower;
  private RelativeEncoder encoder;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(BasicFlexController subsystem_param) {
    m_subsystem = subsystem_param;
    encoder = m_subsystem.motor.getEncoder();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("armPosition",encoder.getPosition());
    commandPower = 0;
    if (RobotContainer.m_driverController.getLeftBumperButton() || RobotContainer.m_driverController2.getLeftBumperButton()) {
      commandPower = -1;
    }
    if (RobotContainer.m_driverController.getRightBumperButton() || RobotContainer.m_driverController2.getRightBumperButton()) {
      commandPower = 1;
    }
    if (!RobotContainer.limit0.get()) {
      commandPower = Math.max(0, commandPower);
      encoder.setPosition(0);
    }
    m_subsystem.go(commandPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.go(0);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
