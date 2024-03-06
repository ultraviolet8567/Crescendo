package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
	private Shooter shooter;
	// private Intake intake;

	private boolean shotComplete;

	public Shoot(Shooter shooter) {
		this.shooter = shooter;
		// this.intake = intake;

		shotComplete = false;
	}

	@Override
	public void initialize() {
		shooter.shoot();
	}

	@Override
	public void execute() {
		shooter.shoot();
		// if (shooter.atVelocity()) {
		// intake.runIndexer();
		// if (!intake.noteDetected) {
		// shotComplete = true;
		// }
		// }
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
		// intake.stop();
	}

	@Override
	public boolean isFinished() {
		return shotComplete;
	}
}
