package frc.robot.commands.autocmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.AbstractController;
import frc.robot.subsystems.BasicFlexController;

public class FeederCommand extends Command {
    protected final AbstractController m_subsystem;
    private double commandPower;
    public FeederCommand(AbstractController subsystem_param, float commandPower) {
        this.commandPower = commandPower;
        m_subsystem = subsystem_param;
    }

    @Override
    public void execute() {
        m_subsystem.go(commandPower);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.go(0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return !RobotContainer.limit1.get();
    }
}
