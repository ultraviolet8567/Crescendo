package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class AutoRetract extends Command {
	private Intake intake;
	private Timer timer;

	public AutoRetract(Intake intake) {
		this.intake = intake;
		timer = new Timer();
	}

	@Override
	public void initialize() {
		timer.start();
		intake.drop();

		System.out.println("Retracting");
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();
	}

	@Override
	public boolean isFinished() {
		return timer.get() >= 0.05;
	}
}
