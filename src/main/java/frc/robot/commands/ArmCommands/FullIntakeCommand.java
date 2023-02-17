package frc.robot.commands.ArmCommands;

import frc.robot.commands.HandCommands.HandInCommandCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullIntakeCommand extends CommandBase {
    private HandSubsystem s_Arm;



    public FullIntakeCommand(HandSubsystem s_Arm) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);

    }

    public void initialize() {
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {
        // Add in command to be executed
   //     new SequentialCommandGroup(
     //   while (s_Arm.getvoltage() < 14){
            new HandInCommandCone(s_Arm);
     //   }
       // new SequentialCommandGroup(

   //     }
        
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
