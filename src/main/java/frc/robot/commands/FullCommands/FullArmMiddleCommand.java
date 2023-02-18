package frc.robot.commands.FullCommands;

import frc.robot.commands.ArmCommands.ArmMiddleCommand;
import frc.robot.commands.ExtendCommands.ArmExtendCommand;
import frc.robot.commands.ExtendCommands.FinalArmOut;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtendingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullArmMiddleCommand extends CommandBase {
    private ArmSubsystem s_Arm;
    private ExtendingSubsystem s_Extend;



    public FullArmMiddleCommand(ArmSubsystem s_Arm, ExtendingSubsystem s_Extend) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);

        this.s_Extend = s_Extend;
        addRequirements(s_Extend);

    }

    public void initialize() {
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {
        // Add in command to be executed
        
        new SequentialCommandGroup(
            new ArmMiddleCommand(s_Arm),
            new ArmExtendCommand(s_Extend)
            
        );

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
