package frc.robot.commands.HandCommands;

import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunThemHandSlowly extends CommandBase {
    private HandSubsystem s_Hand;

    public RunThemHandSlowly(HandSubsystem s_Hand) {
        this.s_Hand = s_Hand;
        addRequirements(s_Hand);
    


    }

    @Override
    public void initialize() {
        m_timer.start();
        m_timer.reset();
    }

    @Override
    public void execute() {
        
            s_Hand.RunHandsInCone();
      
      }    
        

    

    public void end(boolean interrupted) {
        // when command ends, stop motors here
        s_Hand.Stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
