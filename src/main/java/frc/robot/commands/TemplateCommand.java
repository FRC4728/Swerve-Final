package frc.robot.commands;

import frc.robot.subsystems.TemplateSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TemplateCommand extends CommandBase {
    private TemplateSubsystem m_template;

    private BooleanSupplier buttonsupplier;

    public TemplateCommand(TemplateSubsystem m_template, BooleanSupplier buttonsupplier) {
        this.m_template = m_template;
        addRequirements(m_template);

        this.buttonsupplier = buttonsupplier;
    }

    public void initialize() {
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {
        // Add in command to be executed
        boolean button = buttonsupplier.getAsBoolean();

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
