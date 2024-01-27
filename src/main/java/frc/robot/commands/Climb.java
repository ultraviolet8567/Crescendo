package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class Climb extends Command {
	private Climber climb;

	public Climb(Climber climb) {
		this.climb = climb;
	}

	@Override
	public void execute() {
		climb.startClimb();
	}

	@Override
	public void end(boolean interrupted) {
		climb.stopClimb();
	}
}
