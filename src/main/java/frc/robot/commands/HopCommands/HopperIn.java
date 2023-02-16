package frc.robot.commands.HopCommands;

import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HopperIn extends CommandBase {
    private HopperSubsystem s_Hopper;



    public HopperIn(HopperSubsystem s_Hopper) {
        this.s_Hopper = s_Hopper;
        addRequirements(s_Hopper);


    }

    @Override
    public void execute() {
        s_Hopper.HopIn();

        /* Drive */

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
