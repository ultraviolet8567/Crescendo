package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.intake.Intake;

public class Pickup extends Command {
	private Intake intake;

	public Pickup(Intake intake) {
		this.intake = intake;

		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.pickup();
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();

		if (!interrupted) {
			RobotContainer.getOperatorJoystick().setRumble(RumbleType.kBothRumble, 0.025);
		}
	}

	@Override
	public boolean isFinished() {
		return Lights.getInstance().hasNote;
	}
}
