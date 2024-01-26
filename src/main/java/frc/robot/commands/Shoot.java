package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
	private Shooter shooter;

	public double shooterTopVelocity = 0.0;
	public double shooterBottomVelocity = 0.0;

	public Shoot(Shooter shooter) {
		this.shooter = shooter;
	}

	@Override
	public void execute() {
		shooter.shoot(shooterTopVelocity, shooterBottomVelocity);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
	}
}
