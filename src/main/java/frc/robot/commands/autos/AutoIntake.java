package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.intake.Intake;

public class AutoIntake extends Command {
	private Intake intake;
	private Timer timer;

	public AutoIntake(Intake intake) {
		this.intake = intake;
		timer = new Timer();

		addRequirements(intake);
	}

	@Override
	public void initialize() {
		timer.start();
		intake.pickup();
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();
	}

	@Override
	public boolean isFinished() {
		return Math.abs(timer.get() - AutoConstants.kAutoIntakeTime) < 0.1;
	}
}
