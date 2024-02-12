package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
	private Shooter shooter;

	public double shooterTopVelocity = 2;
	public double shooterBottomVelocity = 0.5;

	public Shoot(Shooter shooter) {
		this.shooter = shooter;
	}

	@Override
	public void execute() {
		shooter.shoot(shooterTopVelocity);
		Lights.getInstance().solid(Lights.Section.FULL, Color.kPurple); // Lights thing, just a test for now
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
	}
}
