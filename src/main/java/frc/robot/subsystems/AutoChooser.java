package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class AutoChooser extends VirtualSubsystem {
	private static final ShuffleboardTab main = Shuffleboard.getTab("Main");
	private final SendableChooser<String> sideOfField, noteNumber, otherStuff;

	public AutoChooser() {
		noteNumber = new SendableChooser<>();
		noteNumber.setDefaultOption("0 Note", "Do Nothing");
		noteNumber.addOption("1 Note", "1 Note");
		noteNumber.addOption("2 Note", "2 Note");
		noteNumber.addOption("3 Note", "3 Note");
		noteNumber.addOption("4 Note", "4 Note");
		noteNumber.addOption("5 Note", "5 Note");

		sideOfField = new SendableChooser<>();
		sideOfField.setDefaultOption("Amp", "Amp Side");
		sideOfField.addOption("Source", "Source Side");
		sideOfField.addOption("Center", "Center");

		otherStuff = new SendableChooser<>();
		otherStuff.setDefaultOption("None", "");
		otherStuff.addOption("Test", "Test");
		otherStuff.addOption("Rush", "Rush ");
		otherStuff.addOption("Inner", "Inner ");
		otherStuff.addOption("Shark", "Shark ");
		otherStuff.addOption("Out of Way", "Out Of Way ");
		otherStuff.addOption("Don't Move", "Don't Move ");

		// Post the selectors to the ShuffleBoard
		main.add("Number of Notes", noteNumber).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1)
				.withPosition(3, 0);
		main.add("Side of Field", sideOfField).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1)
				.withPosition(3, 1);
		main.add("Other Variables", otherStuff).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1)
				.withPosition(3, 2);
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Auto/Routine", getAutoCommand());
	}

	/**
	 * Returns name of pre-defined autonomous command based on Shuffleboard input
	 */
	public String getAutoCommand() {
		if (noteNumber.getSelected().equals("Do Nothing")) {
			return "Do Nothing";
		} else if (otherStuff.getSelected().equals("Test")) {
			return "Test";
		} else {
			return noteNumber.getSelected() + " " + otherStuff.getSelected() + sideOfField.getSelected();
		}
	}
}
