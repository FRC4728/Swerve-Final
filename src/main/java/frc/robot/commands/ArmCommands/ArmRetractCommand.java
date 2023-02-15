package frc.robot.commands.ArmCommands;


import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRetractCommand extends CommandBase {
    private ArmSubsystem s_Arm;



    public ArmRetractCommand(ArmSubsystem s_Arm) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);

    }

    public void initialize() {
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {
        // Add in command to be executed
s_Arm.Extend();
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
