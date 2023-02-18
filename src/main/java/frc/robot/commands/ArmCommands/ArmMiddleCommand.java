package frc.robot.commands.ArmCommands;


import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmMiddleCommand extends CommandBase {
    private ArmSubsystem s_Arm;



    public ArmMiddleCommand(ArmSubsystem s_Arm) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);


    }

    @Override
    public void execute() {
        s_Arm.ActuateMiddle();

        /* Drive */

    }

    public void end(boolean interrupted) {
        // when command ends, stop motors here
        s_Arm.Stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
