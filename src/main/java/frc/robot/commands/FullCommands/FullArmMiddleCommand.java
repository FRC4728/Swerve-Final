package frc.robot.commands.FullCommands;

import frc.robot.commands.ArmCommands.ArmMiddleCommand;
import frc.robot.commands.ArmCommands.ArmToHomeCommand;
import frc.robot.commands.ExtendCommands.ArmExtendCommand;
import frc.robot.commands.ExtendCommands.ArmRetractCommand;
import frc.robot.commands.HandCommands.HandOutConeCommand;
import frc.robot.commands.HandCommands.HandOutCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtendingSubsystem;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FullArmMiddleCommand extends CommandBase {
    private ArmSubsystem s_Arm;
    private ExtendingSubsystem s_Extend;
    private HandSubsystem s_Hand;



    public FullArmMiddleCommand(ArmSubsystem s_Arm, ExtendingSubsystem s_Extend, HandSubsystem s_Hand) {
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
        
    //    if (SmartDashboard.getBoolean("IsCone?", true) == true) {
            new SequentialCommandGroup(
              //  new ArmRetractCommand(s_Extend),
                new ArmMiddleCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() < 109.8) &  (s_Arm.getEncoderActuate() > 100.2)),
                //new ArmExtendCommand(s_Extend),
                //new ParallelRaceGroup(
                    //new HandOutConeCommand(s_Hand),
                    //new WaitCommand(1)
                //),
                //new ArmRetractCommand(s_Extend),
                new ArmToHomeCommand(s_Arm).until (() -> (s_Arm.getEncoderActuate() < 7.5) & (s_Arm.getEncoderActuate() > 2.5))
            );
     //   }

         /*    else if (SmartDashboard.getBoolean("IsCone?", true ) == false){
                new SequentialCommandGroup(
                    new ArmRetractCommand(s_Extend),
                    new ArmMiddleCommand(s_Arm),
                    new ArmExtendCommand(s_Extend),
                    new HandOutCubeCommand(s_Hand),
                    new ArmRetractCommand(s_Extend),
                    new ArmToHomeCommand(s_Arm)
                );
            }*/

            
        

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
