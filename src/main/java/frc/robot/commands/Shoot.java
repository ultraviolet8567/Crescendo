package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
	private Shooter shooter;
	private Intake intake;

	public Shoot(Shooter shooter, Intake intake) {
		this.shooter = shooter;
		this.intake = intake;

		addRequirements(shooter, intake);
	}

	@Override
	public void initialize() {
		shooter.shoot();
	}

	@Override
	public void execute() {
		if (shooter.atVelocity()) {
			intake.runIndexer();
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
		intake.stop();
	}
}
