package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Lights extends VirtualSubsystem {
	private static Lights instance;

	public static Lights getInstance() {
		if (instance == null)
			instance = new Lights();
		return instance;
	}

	// Robot state tracking
	public int loopCycleCount = 0;
	public boolean lowBattery = false;
	public boolean hasNote = false;
	public RobotState state = RobotState.DISABLED;

	// LED IO
	private final AddressableLED leds;
	private final AddressableLEDBuffer buffer;

	// Constants
	private static final int leftLength = 19;
	private static final int rightLength = 19;
	private static final int backLength = 13;
	private static final int frontLength = rightLength + leftLength;
	private static final int length = 86;
	private static final int bottomLength = 8;
	private static final int minLoopCycleCount = 10;
	private static final double lowBatteryVoltage = 10.0;
	private static final double shimmerExtremeness = 0.5;
	private static final double shimmerSpeed = 1;
	private static final double strobeTickSkip = 15;
	private static final int strobeSlowDuration = 5;
	private static final double stripeDuration = 0.75;
	private static final int stripeLength = 5;
	private static final double breathDuration = 1.0;
	private static final double waveExponent = 0.4;
	private static final int lowBatteryFlashWait = 50;
	private static final int lowBatteryFlashDuration = 25;
	private static final int strobeFastDuration = 2;
	private static final double rainbowCycleLength = 25.0;
	private static final double rainbowDuration = 0.25;
	private static final double waveFastCycleLength = 25.0;
	private static final double waveFastDuration = 0.25;
	private static final double waveSlowCycleLength = 25.0;
	private static final double waveSlowDuration = 3.0;
	private static final double waveAllianceCycleLength = 15.0;
	private static final double waveAllianceDuration = 2.0;

	private Lights() {
		leds = new AddressableLED(1);
		buffer = new AddressableLEDBuffer(length);

		leds.setLength(length);

		if (Constants.lightsExist) {
			System.out.println("[Init] Creating Lights");

			leds.setData(buffer);
			leds.start();
		} else {
			System.out.println("[Init] Lights do not exist");
		}
	}

	public void periodic() {
		Logger.recordOutput("RobotState/HasNote", hasNote);

		if (Constants.lightsExist) {
			// Exit during initial cycles
			loopCycleCount++;
			if (loopCycleCount < minLoopCycleCount) {
				return;
			}

			// First branch off depending on what part of the match the robot is in

			// Disabled
			if (state == RobotState.DISABLED) {
				// Purple and yellow stripes
				stripes(Section.FULL, List.of(Color.kPurple, Color.kDarkGoldenrod), stripeLength, stripeDuration);
			}

			// Autonomous
			else if (state == RobotState.AUTO) {
				// Rainbow
				rainbow(Section.FULL);
			}

			// Teleop
			else {
				// Alliance colors
				if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
					wave(Section.FULL, Color.kRoyalBlue, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration);
					// shimmer(Section.FULL, Color.kRoyalBlue);
				} else {
					wave(Section.FULL, Color.kPaleVioletRed, Color.kRed, waveSlowCycleLength, waveSlowDuration);
					// shimmer(Section.FULL, Color.kPaleVioletRed);
				}

				// Pickup indicator
				if (hasNote) {
					solid(Section.FULL, Color.kGreen);
				}
			}

			// Indicate low battery in every case
			lowBattery = (RobotController.getBatteryVoltage() < lowBatteryVoltage);
			if (lowBattery) {
				strobe(Section.BOTTOM, Color.kRed);
			}

			// Update LEDs
			leds.setData(buffer);
		}
	}

	public void solid(Section section, Color color) {
		for (int i = section.start(); i < section.end(); i++) {
			buffer.setLED(i, color);
		}
	}

	private void shimmer(Section section, Color color) {
		if (section == Section.BOTTOM) {
			shimmer(Section.LEFTBOTTOM, color);
			shimmer(Section.RIGHTBOTTOM, color);
		} else {
			for (int i = section.start(); i < section.end(); i++) {
				double brightnessFactor = shimmerExtremeness + Math.sin((loopCycleCount + i) * 0.01) * shimmerSpeed;
				buffer.setLED(i, new Color(color.red * brightnessFactor, color.green * brightnessFactor,
						color.blue * brightnessFactor));
			}
		}
	}

	private void rainbow(Section section) {
		if (section == Section.BOTTOM) {
			rainbow(Section.LEFTBOTTOM);
			rainbow(Section.RIGHTBOTTOM);
		} else {
			for (int i = section.start(); i < section.end(); i++) {
				int hue = ((loopCycleCount * 3) % 180 + (i * 180 / leftLength)) % 180;
				buffer.setHSV(i, hue, 255, 128);
			}
		}
	}

	private void strobe(Section section, Color color) {
		if (section == Section.BOTTOM) {
			strobe(Section.LEFTBOTTOM, color);
			strobe(Section.RIGHTBOTTOM, color);
		} else {
			for (int i = section.start(); i < section.end(); i++) {
				if (loopCycleCount % (strobeTickSkip) < strobeSlowDuration) {
					buffer.setLED(i, color);
				} else {
					buffer.setHSV(i, 0, 0, 0);
				}
			}
		}
	}

	private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
		double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
		double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
		for (int i = 0; i < section.end(); i++) {
			x += xDiffPerLed;
			if (i >= section.start()) {
				double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
				if (Double.isNaN(ratio)) {
					ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
				}
				if (Double.isNaN(ratio)) {
					ratio = 0.5;
				}
				double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
				double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
				double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
				buffer.setLED(i, new Color(red, green, blue));
			}
		}
	}

	private void stripes(Section section, List<Color> colors, int frontLength, double duration) {
		int offset = (int) (Timer.getFPGATimestamp() % duration / duration * frontLength * colors.size());
		for (int i = section.start(); i < section.end(); i++) {
			int colorIndex = (int) (Math.floor((double) (i - offset) / frontLength) + colors.size()) % colors.size();
			colorIndex = colors.size() - 1 - colorIndex;
			buffer.setLED(i, colors.get(colorIndex));
		}
	}

	private void breath(Section section, Color c1, Color c2, double duration) {
		breath(section, c1, c2, duration, Timer.getFPGATimestamp());
	}

	private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
		double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
		double ratio = (Math.sin(x) + 1.0) / 2.0;
		double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
		double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
		double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
		solid(section, new Color(red, green, blue));
	}

	public static enum Section {
		FULL, BOTTOM, UPPER, LEFTUPPER, LEFTBOTTOM, LEFTFULL, RIGHTUPPER, RIGHTBOTTOM, RIGHTFULL, LEFTSMALL, RIGHTSMALL, LEFTBACK, RIGHTBACK;

		private int start() {
			switch (this) {
				case FULL :
					return 0;
				case LEFTUPPER :
					return bottomLength;
				case LEFTBOTTOM :
					return 0;
				case LEFTFULL :
					return 0;
				case RIGHTUPPER :
					return bottomLength + leftLength;
				case RIGHTBOTTOM :
					return 1 + leftLength;
				case RIGHTFULL :
					return 0 + leftLength;
				case LEFTBACK :
					return 1 + frontLength;
				case RIGHTBACK :
					return 1 + frontLength + backLength;
				default :
					return 0;
			}
		}

		private int end() {
			switch (this) {
				case FULL :
					return length;
				case LEFTUPPER :
					return leftLength;
				case LEFTBOTTOM :
					return bottomLength;
				case LEFTFULL :
					return leftLength;
				case RIGHTUPPER :
					return frontLength;
				case RIGHTBOTTOM :
					return leftLength + bottomLength + 1;
				case RIGHTFULL :
					return frontLength;
				case LEFTBACK :
					return frontLength + backLength;
				case RIGHTBACK :
					return frontLength + backLength + backLength;
				default :
					return 0;
			}
		}
	}

	public static enum RobotState {
		DISABLED, AUTO, TELEOP;
	}
}
