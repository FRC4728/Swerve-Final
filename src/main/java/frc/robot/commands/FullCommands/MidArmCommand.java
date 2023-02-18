package frc.robot.commands.FullCommands;

import frc.robot.commands.ExtendCommands.FinalArmOut;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MidArmCommand extends CommandBase {
    private ArmSubsystem s_Arm;



    public MidArmCommand(ArmSubsystem s_Arm) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);

    }

    public void initialize() {
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {
        // Add in command to be executed
        
        new SequentialCommandGroup(
        //    new ArmUpCommand(s_Arm);
         //   new ArmExtendCommand(s_Arm) 
            
            
            );
        if (s_Arm.getEncoderActuate() > 15) {
            new FinalArmOut(s_Arm);
        }

    }

    public void end(boolean interrupted) {
        // when command ends, stop motors here
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
