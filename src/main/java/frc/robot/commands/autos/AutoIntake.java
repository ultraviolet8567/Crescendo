package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.intake.Intake;

public class AutoIntake extends Command {
	private Intake intake;
	private Timer timer;

	public AutoIntake(Intake intake) {
		this.intake = intake;
		timer = new Timer();
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		intake.pickup();

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
