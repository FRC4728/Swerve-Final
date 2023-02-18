package frc.robot.commands.FullCommands;

import frc.robot.commands.ArmCommands.ArmToHomeCommand;
import frc.robot.commands.ArmCommands.ArmToHopperCommand;
import frc.robot.commands.ExtendCommands.ArmRetractCommand;
import frc.robot.commands.HandCommands.HandInConeCommand;
import frc.robot.commands.HandCommands.HandInCubeCommand;
import frc.robot.commands.HandCommands.RunThemHandSlowly;
import frc.robot.commands.HopCommands.HopperIn;
import frc.robot.commands.HopCommands.HopperOut;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtendingSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullIntakeCubeCommand extends CommandBase {
    private ArmSubsystem s_Arm;
    private ExtendingSubsystem s_Extend;
    private HandSubsystem s_Hand;
    private HopperSubsystem s_Hop;



    public FullIntakeCubeCommand(ArmSubsystem s_Arm, ExtendingSubsystem s_Extend, HandSubsystem s_Hand, HopperSubsystem s_Hop) {
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
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {
        // Add in command to be executed
        new SequentialCommandGroup(

            new HandInCubeCommand(s_Hand),
            new RunThemHandSlowly(s_Hand),
            new ArmToHomeCommand(s_Arm),
            new HopperIn(s_Hop)

       

        
        );


        
    }

    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("IsCone?", false);
        // when command ends, stop motors here
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}