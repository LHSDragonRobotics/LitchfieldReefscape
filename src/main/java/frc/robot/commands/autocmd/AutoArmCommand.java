package frc.robot.commands.autocmd;

import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.BasicFlexController;

public class AutoArmCommand extends ArmCommand {
    public AutoArmCommand(BasicFlexController subsystem_param, Level level) {
        super(subsystem_param);
        targetLevel = level;
    }

    @Override
    public boolean isFinished() {
        return targetLevel == Level.NONE;
    }

}
