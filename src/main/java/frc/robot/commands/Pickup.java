package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.intake.Intake;

public class Pickup extends Command {
	private Intake intake;

	public Pickup(Intake intake) {
		this.intake = intake;
	}

	@Override
	public void execute() {

		intake.pickup();
		Lights.getInstance().solid(Lights.Section.FULL, Color.kOrange);
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();
	}
}
