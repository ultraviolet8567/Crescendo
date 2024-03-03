package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
	private Shooter shooter;

	public Shoot(Shooter shooter) {
		this.shooter = shooter;
	}

	@Override
	public void execute() {
		shooter.shoot();
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
	}
}
