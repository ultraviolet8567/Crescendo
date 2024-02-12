package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class OverrideNote extends Command {
	Intake intake;

	public OverrideNote(Intake intake) {
		this.intake = intake;
	}

	@Override
	public void execute() {
		intake.hasNote = true;
	}
}