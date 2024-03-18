package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

/**
 * Utility class to configure a Spark object (either SparkMax or SparkFlex) with
 * standard settings
 */
public class SparkConfig {
	/**
	 * Configures the standard settings for a Spark object
	 *
	 * @param spark
	 *            Spark object
	 * @param type
	 *            Type of Spark object
	 */
	public static void config(CANSparkBase spark, SparkType type, IdleMode idleMode) {
		spark.restoreFactoryDefaults();
		spark.setSmartCurrentLimit(type.getCurrentLimit());
		spark.enableVoltageCompensation(120.0);
		spark.setIdleMode(idleMode);
		spark.burnFlash();
	}

	public static void config(CANSparkBase spark, SparkType type) {
		config(spark, type, IdleMode.kBrake);
	}

	public enum SparkType {
		kSparkMax, kSparkFlex;

		public int getCurrentLimit() {
			switch (this) {
				case kSparkFlex :
					return 60;
				case kSparkMax :
					return 40;
				default :
					return 40;
			}
		}
	}

}
