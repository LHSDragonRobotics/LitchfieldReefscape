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
import org.opencv.core.Mat;

/** An example command that uses an example subsystem. */
public class ArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  protected final BasicFlexController m_subsystem;
  private double commandPower;
  private RelativeEncoder encoder;
  protected Level targetLevel = Level.NONE;


  /**
   * Creates a new ArmCommand.
   *
   * @param subsystem_param The subsystem used by this command.
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
    float position = (float) encoder.getPosition();
    SmartDashboard.putNumber("armPosition", position);
    SmartDashboard.putString("armTarget",targetLevel.toString());
    Level up = getNextUp(position);
    Level down = getNextDown(position);
    SmartDashboard.putString("armTargetUp",up.toString());
    SmartDashboard.putString("armTargetDown",down.toString());
    SmartDashboard.putNumber("armTargetHeight",targetLevel.getHeight());
    commandPower = 0;
    if (RobotContainer.m_driverController.getPOV() == 0 || RobotContainer.m_driverController2.getPOV() == 0) {
      commandPower = 1;
      targetLevel = Level.NONE;
    }
    if (RobotContainer.m_driverController.getPOV() == 180 || RobotContainer.m_driverController2.getPOV() == 180) {
      commandPower = -1;
      targetLevel = Level.NONE;

    }
    if (RobotContainer.m_driverController.getLeftBumperButton() || RobotContainer.m_driverController2.getLeftBumperButton()) {
      targetLevel = down;
    }
    if (RobotContainer.m_driverController.getRightBumperButton() || RobotContainer.m_driverController2.getRightBumperButton()) {
      targetLevel = up;
    }

    if (targetLevel != Level.NONE) {
      float appliedPower = (targetLevel.getHeight()-position) / 15f;
      if (Math.abs(appliedPower) < 0.2f) {
        targetLevel = Level.NONE;
      } else {
        commandPower = appliedPower;
      }
    }
    if (!RobotContainer.m_driverController2.getRightStickButton() || targetLevel == Level.BOTTOM) {
      float slowUp = (float) Math.min(1, (float) 179 - position / -25f);
      float slowDown = (float) Math.min(1, (position / 15f + 0.1f));
      SmartDashboard.putNumber("slowUp",slowUp);
      if (commandPower < 0) {
        commandPower *= slowDown;
      }
      if (commandPower > 0) {
        commandPower *= slowUp;
      }
    }
    if (!RobotContainer.limit0.get()) {
      commandPower = Math.max(0, commandPower);
      encoder.setPosition(0);
    }
    if (position > 179) {
      commandPower = Math.min(0, commandPower);
    }
    SmartDashboard.putNumber("commandPower",commandPower);
    m_subsystem.go(commandPower);
  }

  public Level getNextUp(float position) {
    Level best = Level.NONE;
    float distance = 999;
    for (Level level : Level.values()) {
      if (level != Level.NONE) {
        float ourDistance = level.getHeight()-position;
        if (ourDistance > 5f && ourDistance < distance) {
          best = level;
          distance = ourDistance;
        }
      }
    }
    return best;
  }

  public Level getNextDown(float position) {
    Level best = Level.NONE;
    float distance = -999;
    for (Level level : Level.values()) {
      if (level != Level.NONE) {
        float ourDistance = level.getHeight()-position;
        if (ourDistance < -5f && ourDistance > distance) {
          best = level;
          distance = ourDistance;
        }
      }
    }
    return best;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    targetLevel = Level.NONE;
    m_subsystem.go(0);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum Level {
    NONE(0f),
    BOTTOM(0f),
    L1(78.5f),
    L2(170f);

    public float height = 0;

    Level(Float height) {
      this.height = height;
    }

    public float getHeight() {
      return height;
    }
  }
}
