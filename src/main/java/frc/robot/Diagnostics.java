package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class Diagnostics {
    public static boolean armError;
    public static double sensorRange = 0;
    public static Field2d field2d = new Field2d();

    public static void updateErrors() {
        SmartDashboard.putBoolean("armOK", !armError);
    }
    public static void gatherSensorData() {
        SmartDashboard.putBoolean("hasCoral",!RobotContainer.limit1.get());
        SmartDashboard.putNumber("feederAngle",RobotContainer.rotFeeder.getEncoder().getPosition());
    }
    public static void updateDashboard() {
        SmartDashboard.putNumber("gyro", DriveSubsystem.m_gyro.getAngle());
        SmartDashboard.putNumber("sensorRange", sensorRange);
//        SmartDashboard.putNumber("voltage", PowerDistribution.);
        if (SmartDashboard.getBoolean("calibrate", false)) {
            DriveSubsystem.m_gyro.calibrate();
            SmartDashboard.putBoolean("calibrate", false);
        }
        field2d.setRobotPose(RobotContainer.m_robotDrive.getPose());
        SmartDashboard.putData("Field",field2d);
    }
    public static void armError() {
        armError = true;
    }
}