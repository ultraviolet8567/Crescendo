package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class AutoRampShooter extends Command {
	private Shooter shooter;
	private double rpm;

	public AutoRampShooter(Shooter shooter, double rpm) {
		this.shooter = shooter;
		this.rpm = rpm;
	}

	@Override
	public void initialize() {
		shooter.autoShoot(rpm);

		System.out.println("Ramping shooter");
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
	}
}
