// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RobotDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  public double divrate = 0.01;
  public double rotrate = 0.1;
  public float speed = 0.1f;

  
  private XboxController xbox = RobotContainer.m_driverController;
  private XboxController xbox2 = RobotContainer.m_driverController2;
  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  static HashMap<Integer,Integer> dists = new HashMap<>();
  public static void fillDists() { //Fills hashmap with enum values
    dists.clear();
    for (TagDist tag : TagDist.values()) {
        dists.putIfAbsent(tag.id, tag.dist);
    }
  }
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RobotDrive() {
    m_subsystem = RobotContainer.m_robotDrive ;
    // Use addRequirements() here to declare subsystem dependencies.
    fillDists();
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("isAuto", DriverStation.isAutonomous());
    SmartDashboard.putBoolean("isTele", !DriverStation.isTeleopEnabled());
    double tx = tableInstance.getTable("limelight").getEntry("tx").getDouble(0);
    double ta = tableInstance.getTable("limelight").getEntry("ta").getDouble(0);
    double[] translation = new double[3];
    translation = tableInstance.getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    double rightTrigger = xbox.getLeftTriggerAxis();
      divrate = Math.max(0,(speed/(rightTrigger+.1)));
      rotrate = Math.max(0,(speed/(rightTrigger+.1)));

    double zRate = xbox.getRightX() * rotrate;
    // System.out.println(String.valueOf(divrate));

    double JS_BIAS_X = 0; //  double JS_BIAS_X = .3 
    double angleRadians = Math.toRadians(DriveSubsystem.m_gyro.getAngle());

    // compute a deadband-safe speed for each direction of control.
    double xrate = (xbox.getLeftX()*Math.cos(angleRadians)) - xbox.getLeftY()*Math.sin(angleRadians);  //double xrate = xbox.getLeftX() + JS_BIAS_X ;
    xrate *= Math.abs(xrate * divrate); // competition rate is .8 - The lower the decimal the slower it drives
    double yrate = (xbox.getLeftY()*Math.cos(angleRadians)) + xbox.getLeftX()*Math.sin(angleRadians);
    yrate *= Math.abs(yrate * divrate); // competition rate is .8 - The lower the decimal the slower it drives
    /*if (xbox.getYButton() && trans[4] != 0.0d) {
      xrate = ((trans[4])/-80)*rotrate;
      zRate = (tx/80)*rotrate;
    }
    */
    
    if (xbox.getAButton() && ta != 0.0) {
      xrate = ((translation[4])/-80)*rotrate;
      zRate = tx/80;
      yrate = ((ta-1)/2)*rotrate;
    }
    
    if (Math.abs(xbox2.getRightY()) > .1) {
      RobotContainer.rotFeeder.go(-xbox2.getRightY());
    } else {
      RobotContainer.rotFeeder.go(0);
    }
    
    if (Math.abs(xbox2.getLeftY()) > .1) {
      RobotContainer.rotBall.go(xbox2.getLeftY()/3);
    } else {
      RobotContainer.rotBall.go(0);
    }

    float feeder = 0f;

    if (Math.abs(xbox2.getRightTriggerAxis()) > .1) {
      feeder += xbox2.getRightTriggerAxis();
    }
    if (Math.abs(xbox2.getLeftTriggerAxis()) > .1) {
      feeder -= xbox2.getLeftTriggerAxis();
    }
    feeder /= 3;
    RobotContainer.feeder.go(feeder);

     m_subsystem.drive(-yrate, -xrate, -zRate, false, true);
  }
  public static int getDist(int id)  {
    if (dists.containsKey(id)) {
      return dists.get(id);
    } else {
        System.out.println("April tag value not defined!") ;
        return 100;
    }
  }
  public void lockOnTag(float rotrate) {
    double tx = tableInstance.getTable("limelight").getEntry("tx").getDouble(0);
    double ta = tableInstance.getTable("limelight").getEntry("ta").getDouble(0);
    double tagID = tableInstance.getTable("limelight").getEntry("tid").getDouble(0);
    double[] translation = new double[3];
    translation = tableInstance.getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    double xrate = ((translation[4])/-80)*rotrate;
    double zRate = tx/getDist((int)tagID);
    double yrate = ((ta-1)/2)*rotrate;
    m_subsystem.drive(-yrate, -xrate, -zRate, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum TagDist { // Distance values for april tags
    ONE(1,100),
    TWO(2,100),
    THREE(3,100),
    FOUR(4,100),
    FIVE(5,100),
    SIX(6,100);

    public int getID() {
      System.out.println("Value not defined!") ;
      return 100;
    }
    public int dist;
    public int id;
    TagDist(int id, int distance) { this.id = id; this.dist = distance;}
  }
}
