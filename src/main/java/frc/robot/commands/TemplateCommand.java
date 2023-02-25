package frc.robot.commands;

import frc.robot.subsystems.TemplateSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TemplateCommand extends CommandBase {

    public TemplateCommand(TemplateSubsystem m_template, BooleanSupplier buttonsupplier) {
        addRequirements(m_template);
    }

    public void initialize() {
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {

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
