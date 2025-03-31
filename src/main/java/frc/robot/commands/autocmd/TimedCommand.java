 package frc.robot.commands.autocmd;

 import edu.wpi.first.wpilibj.Timer;
 import edu.wpi.first.wpilibj2.command.Command;
 import frc.robot.RobotContainer;
 import frc.robot.subsystems.AbstractController;

 public class TimedCommand extends Command {
     double timeStarted;
     double rate;
     boolean reverse = false;
     double timeToStop;
     AbstractController subsystem;

     public TimedCommand() {
        addRequirements(RobotContainer.feeder);
     }
     public TimedCommand(AbstractController subsystem, double time, double rate) {
       timeToStop = time;
       reverse = true;
       this.rate = rate;
       this.subsystem = subsystem;
     }
     @Override
     public void initialize() {
         timeStarted = Timer.getFPGATimestamp();
         System.out.println("Sucking!");
     }
     @Override
     public void execute() {
         subsystem.go(rate);
     }

     // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
       subsystem.go(0);
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return Timer.getFPGATimestamp() - timeStarted > timeToStop;
   }
 }
