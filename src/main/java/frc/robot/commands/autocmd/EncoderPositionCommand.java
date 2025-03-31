package frc.robot.commands.autocmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AbstractController;

public class EncoderPositionCommand extends Command {

    protected final AbstractController m_subsystem;
    private final double targetAngle;
    private double measuredAngle = 0;
    public EncoderPositionCommand(AbstractController subsystem_param, float angle) {
        this.targetAngle = angle;
        m_subsystem = subsystem_param;
    }

    @Override
    public void execute() {
        measuredAngle = m_subsystem.getEncoder().getPosition();
        double rotFeeder = (0-targetAngle) / 15f;

        double appliedPower = (targetAngle-measuredAngle) / 15f;
        boolean directionFlag= false;
        if (appliedPower > 0 ) {
            appliedPower = ((-230)-measuredAngle) / 15f;
            directionFlag = true;
        }

        if (appliedPower < 0.8f ) {
            if ((rotFeeder > 0 && !directionFlag) || (rotFeeder < 0 && directionFlag)) {
                rotFeeder = (appliedPower);
            }
        }
        m_subsystem.go(rotFeeder);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.go(0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(measuredAngle-targetAngle) < 20;
    }
}
