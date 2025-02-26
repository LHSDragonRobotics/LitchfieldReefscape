package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class Diagnostics {
    public static boolean armError;
    public static double sensorRange = 0;

    public static void updateErrors() {
        SmartDashboard.putBoolean("armOK", !armError);
    }
    public static void gatherSensorData() {
        sensorRange = RobotContainer.colorSensor.getProximity();
    }
    public static void updateDashboard() {
        SmartDashboard.putNumber("gyro", DriveSubsystem.m_gyro.getAngle());
        SmartDashboard.putNumber("sensorRange", sensorRange);
        if (SmartDashboard.getBoolean("calibrate", false)) {
            DriveSubsystem.m_gyro.calibrate();
            SmartDashboard.putBoolean("calibrate", false);
        }
    }
    public static void armError() {
        armError = true;
    }
}