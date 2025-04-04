// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand;
//import frc.robot.commands.ArmTeleCommand;
import frc.robot.commands.BasicCommand;
import frc.robot.commands.RobotDrive;
import frc.robot.commands.autocmd.AutoArmCommand;
import frc.robot.commands.autocmd.EncoderPositionCommand;
import frc.robot.commands.autocmd.FeederCommand;
import frc.robot.commands.autocmd.TimedCommand;
import frc.robot.subsystems.BasicController;
import frc.robot.subsystems.BasicFlexController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public static final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // Motor controller commands
    // public static final BasicController arm = new BasicController(9,false);
    //public static final BasicController claw = new BasicController(11);
    // public static final BasicController sucker = new BasicController(10,false);
    // public static final DualMotorController thrower = new DualMotorController(11, 12, true);
    public static final BasicFlexController arm = new BasicFlexController(new SparkFlex(13, SparkFlex.MotorType.kBrushless));
    public static final BasicController feeder = new BasicController(new SparkMax(12, SparkMax.MotorType.kBrushless));
    public static final BasicController rotFeeder = new BasicController(new SparkMax(11, SparkMax.MotorType.kBrushless));
    public static final BasicFlexController ball = new BasicFlexController(new SparkFlex(14, SparkFlex.MotorType.kBrushless));
    public static final BasicController rotBall = new BasicController(new SparkMax(10, SparkMax.MotorType.kBrushless));
//    public static final PowerDistribution power = new PowerDistribution()

    public static final DigitalInput limit0 = new DigitalInput(0);
    public static final DigitalInput limit1 = new DigitalInput(1);
    // public static final DigitalInput limit1 = new DigitalInput(1);

    SendableChooser<String> autoSelect = new SendableChooser<>();

    //public static final WPI_TalonSRX armEncoder = new WPI_TalonSRX(14);

    // The driver's controller
    public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    public static XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort2);


    public static boolean followPathPlanner = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        NamedCommands.registerCommand("L0", new AutoArmCommand(arm, ArmCommand.Level.BOTTOM)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        NamedCommands.registerCommand("L1", new AutoArmCommand(arm, ArmCommand.Level.L1)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        NamedCommands.registerCommand("L2", new AutoArmCommand(arm, ArmCommand.Level.L2)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        NamedCommands.registerCommand("feed", new FeederCommand(feeder, .5f)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        NamedCommands.registerCommand("backFeed", new TimedCommand(feeder,1.5f, -.5f)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        NamedCommands.registerCommand("rotFeed", new EncoderPositionCommand(rotFeeder,-40)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        NamedCommands.registerCommand("rotL1", new EncoderPositionCommand(rotFeeder,-220)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        NamedCommands.registerCommand("rotDescore", new EncoderPositionCommand(rotFeeder,-180


        )/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        // NamedCommands.registerCommand("intake", new IntakeCommand()/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        // NamedCommands.registerCommand("backfeed", new IntakeCommand(.25,-.3)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        // First option is dropdown Name, 2nd is actual name in file!
        autoSelect.setDefaultOption("Path", "path");
        File directory = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
        String[] files = directory.list();
        assert files != null;
        for (String string : files) {
            File file = new File(Filesystem.getDeployDirectory(),"pathplanner/autos/"+string);
            String fileName = file.getName();
            int dotIndex = fileName.lastIndexOf(".");
            if (dotIndex == -1) {
                break;
            }
            autoSelect.addOption(fileName.substring(0,dotIndex),fileName.substring(0,dotIndex));
        }


        SmartDashboard.putString("Auto Selector", "path");
        SmartDashboard.putBoolean("SwapSide", false);
        SmartDashboard.putBoolean("isAuto", DriverStation.isAutonomous());
        SmartDashboard.putBoolean("isTele", !DriverStation.isTeleopEnabled());

        // Configure the button bindings
        SmartDashboard.putData(autoSelect);
        configureButtonBindings();

        boolean USE_REV_CMD = false;

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.

                (USE_REV_CMD) ?
                        new RunCommand(
                                () -> m_robotDrive.drive(
                                        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                        false, true),
                                m_robotDrive)
                        :
                        Commands.parallel(new RobotDrive(), new ArmCommand(arm))
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, XboxController.Button.kB.value)
                .whileTrue(new BasicCommand(ball, -.5d));
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .whileTrue(new BasicCommand(ball, .5d));
        new JoystickButton(m_driverController2, XboxController.Button.kB.value)
                .whileTrue(new BasicCommand(ball, -.5d));
        new JoystickButton(m_driverController2, XboxController.Button.kX.value)
                .whileTrue(new BasicCommand(ball, .5d));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                AutoBuilder.buildAuto(autoSelect.getSelected()));
        //return new SequentialCommandGroup(new ShootCommand(1), new AutoDrive(1));
        // Autos.exampleAuto();
    }


    public Command UNUSED_EXAMPLE_getAutonomousCommand() {

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }
}