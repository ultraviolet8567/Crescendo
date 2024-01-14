package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or value not in dashboard.
 */
public class LoggedTunableNumber {
	private static final ShuffleboardTab table = Shuffleboard.getTab("TunableNumbers");
	private static final boolean tuningMode = Constants.currentMode == Mode.TUNING;

	private final String key;
	private final GenericEntry entry;
	private final double defaultValue;

	/**
	 * Create a new LoggedTunableNumber
	 *
	 * @param dashboardKey
	 *            Key on dashboard
	 */
	public LoggedTunableNumber(String dashboardKey) {
		this(dashboardKey, 0);
	}

	/**
	 * Create a new LoggedTunableNumber with the default value
	 *
	 * @param dashboardKey
	 *            Key on dashboard
	 * @param defaultValue
	 *            Default value
	 */
	public LoggedTunableNumber(String dashboardKey, double defaultValue) {
		key = dashboardKey;
		entry = table.add(key, defaultValue).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();

		this.defaultValue = defaultValue;
	}

	/**
	 * Get the current value, from dashboard if available and in tuning mode.
	 *
	 * @return The current value
	 */
	public double get() {
		return tuningMode ? entry.getDouble(defaultValue) : defaultValue;
	}

	/**
	 * Get the default value for the number that has been set
	 *
	 * @return The default value
	 */
	public double getDefault() {
		return defaultValue;
	}

	/**
	 * Get the key for the record on the dashboard
	 *
	 * @return The dashboard key
	 */
	public String getKey() {
		return key;
	}
}
