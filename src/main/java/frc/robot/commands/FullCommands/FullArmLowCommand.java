package frc.robot.commands.FullCommands;

import frc.robot.commands.ArmCommands.ArmMiddleCommand;
import frc.robot.commands.ArmCommands.ArmToGroundCommand;
import frc.robot.commands.ArmCommands.ArmToHomeCommand;
import frc.robot.commands.ExtendCommands.ArmExtendCommand;
import frc.robot.commands.ExtendCommands.ArmRetractCommand;
import frc.robot.commands.HandCommands.HandOutConeCommand;
import frc.robot.commands.HandCommands.HandOutCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtendingSubsystem;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullArmLowCommand extends CommandBase {
    private ArmSubsystem s_Arm;
    private ExtendingSubsystem s_Extend;
    private HandSubsystem s_Hand;



    public FullArmLowCommand(ArmSubsystem s_Arm, ExtendingSubsystem s_Extend, HandSubsystem s_Hand) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);

        this.s_Extend = s_Extend;
        addRequirements(s_Extend);

        this.s_Hand = s_Hand;
        addRequirements(s_Hand);

    }

    public void initialize() {
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {
        // Add in command to be executed
        
        if (SmartDashboard.getBoolean("IsCone?", true) == false) {
            new SequentialCommandGroup(
                new ArmRetractCommand(s_Extend),
                new ArmToGroundCommand(s_Arm),
                new HandOutCubeCommand(s_Hand),
                new ArmToHomeCommand(s_Arm)
            );
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
