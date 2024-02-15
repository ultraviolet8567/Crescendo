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
	}

	@Override
	public void execute() {
		intake.runIndexer();
		shooter.shoot();
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();
		shooter.stop();
	}
}
