// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasicController extends AbstractController {

  // Single-motor mechanism....
  public  MotorController motor = null;


  /** Creates a new DriveSubsystem. */
  public BasicController(int CANID, boolean kBrushed) {
    if (kBrushed) {
          motor = new SparkMax(CANID,  SparkMax.MotorType.kBrushed);
    } else {
          motor = new SparkMax(CANID,  SparkMax.MotorType.kBrushless);
    }
  }
  public BasicController(SparkMax sparkMax) {
    motor = sparkMax;
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