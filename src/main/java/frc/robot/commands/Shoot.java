package frc.robot.commands;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
	private Shooter shooter;
	private Intake intake;

	private Timer timer;
	private boolean shotComplete;

	public Shoot(Shooter shooter, Intake intake) {
		this.shooter = shooter;
		this.intake = intake;

		timer = new Timer();
		shotComplete = false;
	}

	@Override
	public void execute() {
		shooter.shoot();
		
		// 87% buffer if the shooter flywheels never actually reach their target
		// velocity
		if (shooter.atVelocity()) {
			intake.runIndexer();
			if (!intake.noteDetected) {
				timer.schedule(new TimerTask() {
					@Override
					public void run() {
						shotComplete = true;
					}
				}, 1000);
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
	}

	@Override
	public boolean isFinished() {
		return shotComplete;
	}
}
