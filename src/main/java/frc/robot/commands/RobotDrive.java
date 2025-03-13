// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

import java.util.HashMap;
import java.util.IllegalFormatCodePointException;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.opencv.core.Mat;

/**
 * An example command that uses an example subsystem.
 */
public class RobotDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_subsystem;
    public double divrate = 0.01;
    public double rotrate = 0.1;
    public float speed = 0.1f;
    private float rotBallLimit = (float) RobotContainer.rotBall.motor.getEncoder().getPosition();
    private double lastAngle = DriveSubsystem.m_gyro.getAngle();

    private XboxController xbox = RobotContainer.m_driverController;
    private XboxController xbox2 = RobotContainer.m_driverController2;
    private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    static HashMap<Integer, Integer> dists = new HashMap<>();

    public static void fillDists() { //Fills hashmap with enum values
        dists.clear();
        for (TagDist tag : TagDist.values()) {
            dists.putIfAbsent(tag.id, tag.dist);
        }
    }

    /**
     * Creates a new ExampleCommand.
     */
    public RobotDrive() {
        m_subsystem = RobotContainer.m_robotDrive;
        // Use addRequirements() here to declare subsystem dependencies.
        fillDists();
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("isAuto", DriverStation.isAutonomous());
        SmartDashboard.putBoolean("isTele", !DriverStation.isTeleopEnabled());
        double tx = tableInstance.getTable("limelight").getEntry("tx").getDouble(0);
        double ta = tableInstance.getTable("limelight").getEntry("ta").getDouble(0);
        double tagID = tableInstance.getTable("limelight").getEntry("tid").getDouble(0);
        double[] translation = new double[3];
        translation = tableInstance.getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        double rightTrigger = xbox.getLeftTriggerAxis();
        divrate = Math.max(0, (speed / (rightTrigger + .1)));
        rotrate = Math.max(0, (speed / (rightTrigger + .1)));


        double zRate = xbox.getRightX() * rotrate;
        // System.out.println(String.valueOf(divrate));

        double JS_BIAS_X = 0; //  double JS_BIAS_X = .3
        double angleRadians = Math.toRadians(DriveSubsystem.m_gyro.getAngle());
        double angleRadiansVelocity = Math.toRadians(lastAngle - DriveSubsystem.m_gyro.getAngle());
        lastAngle = DriveSubsystem.m_gyro.getAngle();

        // compute a deadband-safe speed for each direction of control.
        double xrate = (xbox.getLeftX() * Math.cos(angleRadians)) - xbox.getLeftY() * Math.sin(angleRadians);  //double xrate = xbox.getLeftX() + JS_BIAS_X ;
        xrate *= Math.abs(xrate * divrate); // competition rate is .8 - The lower the decimal the slower it drives
        double yrate = (xbox.getLeftY() * Math.cos(angleRadians)) + xbox.getLeftX() * Math.sin(angleRadians);
        yrate *= Math.abs(yrate * divrate); // competition rate is .8 - The lower the decimal the slower it drives
    /*if (xbox.getYButton() && trans[4] != 0.0d) {
      xrate = ((trans[4])/-80)*rotrate;
      zRate = (tx/80)*rotrate;
    }
    */

        if (xbox.getRightStickButton()) {
            zRate = (angleRadians % (Math.PI * 2)) / (0.6f + Math.max(1, Math.abs(angleRadiansVelocity) * 10));
        }

        if (ta != 0.0) {
            float dist = getDist((int) tagID);
            float zRate2 = (float) ((tx) / 120);
            float xrate2 = (float) (-((translation[4]) / -160) * rotrate);
            float yrate2 = (float) (-((ta - (dist/160F)) / 2) * rotrate);
            float move = (Math.abs(xrate2) + Math.abs(zRate2) + Math.abs(yrate2)) / 3f;
            SmartDashboard.putNumber("move", move);
            SmartDashboard.putNumber("xrate2", xrate2);
            SmartDashboard.putNumber("zRate2", zRate2);
            SmartDashboard.putNumber("yrate2", yrate2);
            SmartDashboard.putNumber("dist", dist);
            if (xbox.getAButton() && move > 0.03f) {
                xrate = xrate2;
                zRate = zRate2;
                yrate = yrate2;
            }
        }
        float rotFeeder = 0;
        if (Math.abs(xbox2.getRightY()) > .1) {
            rotFeeder -= (float) xbox2.getRightY();
        }

        RobotContainer.rotFeeder.go(rotFeeder);


        float rotBall = 0;
        if (Math.abs(xbox2.getLeftY()) > .1) {
            rotBall += (float) (xbox2.getLeftY() / 3);
        }
        float ballPosition = (float) RobotContainer.rotBall.motor.getEncoder().getPosition();
        if (xbox2.getLeftStickButton()) {
            rotBallLimit = ballPosition;
        }
        if (ballPosition > rotBallLimit) {
            rotBall = Math.min(0, rotBall);
        }
        if (ballPosition < rotBallLimit - 18.5f) {
            rotBall = Math.max(0, rotBall);
        }

        SmartDashboard.putNumber("rotBall", ballPosition);
        SmartDashboard.putNumber("rotBallLimit", rotBallLimit);
        RobotContainer.rotBall.go(rotBall);

        float feeder = 0f;
        if (Math.abs(xbox2.getRightTriggerAxis()) > .1) {
            feeder += xbox2.getRightTriggerAxis();
        }
        if (Math.abs(xbox2.getLeftTriggerAxis()) > .1) {
            feeder -= xbox2.getLeftTriggerAxis();
        }
        feeder /= 3;
        RobotContainer.feeder.go(feeder);

        if (xbox.getStartButton() || xbox2.getStartButton()) {
            DriveSubsystem.m_gyro.calibrate();
            SmartDashboard.putBoolean("calibrate", false);
        }

        m_subsystem.drive(-yrate, -xrate, -zRate, false, true);
    }

    public static int getDist(int id) {
        if (dists.containsKey(id)) {
            return dists.get(id);
        } else {
            System.out.println("April tag value not defined! [" + id + "]");
            return 100;
        }
    }

    public void lockOnTag(float rotrate) {
        double tx = tableInstance.getTable("limelight").getEntry("tx").getDouble(0);
        double ta = tableInstance.getTable("limelight").getEntry("ta").getDouble(0);
        double tagID = tableInstance.getTable("limelight").getEntry("tid").getDouble(0);
        double[] translation = new double[3];
        translation = tableInstance.getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        double xrate = ((translation[4]) / -80) * rotrate;
        double zRate = tx / getDist((int) tagID);
        double yrate = ((ta - 1) / 2) * rotrate;
        m_subsystem.drive(-yrate, -xrate, -zRate, false, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public enum TagDist { // Distance values for april tags
        ONE(1, 10),
        TWO(2, 10),
        THREE(3, 10),
        FOUR(4, 10),
        FIVE(5, 10),
        SIX(6, 10);

        public int getID() {
            System.out.println("Value not defined!");
            return 10;
        }

        public int dist;
        public int id;

        TagDist(int id, int distance) {
            this.id = id;
            this.dist = distance;
        }
    }
}
