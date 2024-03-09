package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.intake.Intake;

public class AutoIntakeTimed extends Command {
	private Intake intake;
	private Timer timer;

	public AutoIntakeTimed(Intake intake) {
		this.intake = intake;
		timer = new Timer();

		addRequirements(intake);
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
		return timer.get() > AutoConstants.kAutoIntakeTime;
	}
}
