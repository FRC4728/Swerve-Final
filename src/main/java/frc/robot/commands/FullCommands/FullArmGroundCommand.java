package frc.robot.commands.FullCommands;

import frc.robot.commands.ArmCommands.ArmToGroundCommand;
import frc.robot.commands.ArmCommands.ArmToHomeCommand;
import frc.robot.commands.ArmCommands.ArmToHopperCommand;
import frc.robot.commands.ExtendCommands.ArmRetractCommand;
import frc.robot.commands.ExtendCommands.ExtendToGroundCommand;
import frc.robot.commands.ExtendCommands.PistonArmIn;
import frc.robot.commands.HandCommands.HandInConeCommand;
import frc.robot.commands.HandCommands.HandInCubeCommand;
import frc.robot.commands.HandCommands.RunThemHandSlowly;
import frc.robot.commands.HopCommands.HopperOut;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtendingSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullArmGroundCommand extends CommandBase {
    private ArmSubsystem s_Arm;
    private ExtendingSubsystem s_Extend;
    private HandSubsystem s_Hand;
    private HopperSubsystem s_Hop;

    private Timer m_timer = new Timer();


    public FullArmGroundCommand(ArmSubsystem s_Arm, ExtendingSubsystem s_Extend, HandSubsystem s_Hand, HopperSubsystem s_Hop) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);

        this.s_Extend = s_Extend;
        addRequirements(s_Extend);

        this.s_Hand = s_Hand;
        addRequirements(s_Hand);

        this.s_Hop = s_Hop;
        addRequirements(s_Hop);

    }

    public void initialize() {
        m_timer.start();
        m_timer.reset();
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {
        // Add in command to be executed
        new SequentialCommandGroup(
            
                new ArmRetractCommand(s_Extend),
                new ArmToGroundCommand(s_Arm),
                new ExtendToGroundCommand(s_Extend),
                
                new HandInCubeCommand(s_Hand),
                new ArmRetractCommand(s_Extend),
                new ArmToHomeCommand(s_Arm)

           
                


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
