package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.intake.Intake;

public class AutoIntake extends Command {
	private Intake intake;

	public AutoIntake(Intake intake) {
		this.intake = intake;
	}

	@Override
	public void initialize() {
		intake.pickup(0.8);

		System.out.println("Intaking");
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();
	}

	@Override
	public boolean isFinished() {
		return Lights.getInstance().hasNote;
	}
}
