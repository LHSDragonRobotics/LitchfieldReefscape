// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasicFlexController extends AbstractController {

  // Single-motor mechanism....
  public  SparkFlex motor = null;


  /** Creates a new DriveSubsystem. */
  public BasicFlexController(int CANID, SparkFlex.MotorType brushType) {
          motor = new SparkFlex(CANID,  brushType);
  }
  public BasicFlexController(SparkFlex sparkFlex) {
    motor = sparkFlex;
  }

  @Override
  public void periodic() {
  }

  /**
   */
  @Override
  public void go(double power) {
     motor.set(power);
  }

}