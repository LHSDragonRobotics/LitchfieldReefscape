// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AbstractController;
import frc.robot.subsystems.BasicController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LimitCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  protected final AbstractController m_subsystem;
  private double commandPower;
  private DigitalInput digitalInput;
  private boolean invert;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimitCommand(AbstractController subsystem_param, double power_param, DigitalInput digitalInput, boolean invert) {
    m_subsystem = subsystem_param;
    commandPower = power_param;
    this.digitalInput = digitalInput;
    this.invert = invert;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (invert) {
      if (!digitalInput.get()) {
        m_subsystem.go(commandPower);
      } else {
      m_subsystem.go(0);
      }
    } else {
      if (digitalInput.get()) {
        m_subsystem.go(commandPower);
      } else {
      m_subsystem.go(0);
      }
    }
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
