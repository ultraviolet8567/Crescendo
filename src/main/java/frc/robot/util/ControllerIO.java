package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.OIConstants;

public final class ControllerIO {
	private static final ControllerType controllerDrive = OIConstants.controllerTypeDriver;

	public static int getLeftY() {
		switch (controllerDrive) {
			case XBOX :
				return 1;
			case LOGITECH :
				return 1;
			case JOYSTICK :
				return 1;
			default :
				return 1;
		}
	}

	public static int getLeftX() {
		switch (controllerDrive) {
			case XBOX :
				return 0;
			case LOGITECH :
				return 0;
			case JOYSTICK :
				return 0;
			default :
				return 0;
		}
	}

	public static int getRightY() {
		switch (controllerDrive) {
			case XBOX :
				return 5;
			case LOGITECH :
				return 5;
			case JOYSTICK :
				return 5;
			default :
				return 5;
		}
	}

	public static int getRot() {
		switch (controllerDrive) {
			case XBOX :
				return 4;
			case LOGITECH :
				return 2;
			case JOYSTICK :
				return 2;
			default :
				return 2;
		}
	}

	public static int inversionY() {
		switch (controllerDrive) {
			case XBOX :
				return -1;
			case LOGITECH :
				return -1;
			case JOYSTICK :
				return 1;
			default :
				return -1;
		}
	}

	public static int inversionX() {
		switch (controllerDrive) {
			case XBOX :
				return -1;
			case LOGITECH :
				return -1;
			case JOYSTICK :
				return 1;
			default :
				return -1;
		}
	}

	public static int inversionRot() {
		switch (controllerDrive) {
			case XBOX :
				return 1;
			case LOGITECH :
				return 1;
			case JOYSTICK :
				return 1;
			default :
				return 1;
		}
	}

	public static int getTrigger() {
		switch (controllerDrive) {
			case XBOX :
				return XboxController.Button.kBack.value;
			case LOGITECH :
				return 1;
			case JOYSTICK :
				return 1;
			default :
				return 1;
		}
	}

	public static int getLeftTrigger() {
		switch (controllerDrive) {
			case XBOX :
				return 2;
			case LOGITECH :
				return 2;
			case JOYSTICK :
				return 2;
			default :
				return 2;
		}
	}

	public static int getRightTrigger() {
		switch (controllerDrive) {
			case XBOX :
				return 2;
			case LOGITECH :
				return 2;
			case JOYSTICK :
				return 2;
			default :
				return 2;
		}
	}

	public static int getRightBumper() {
		switch (controllerDrive) {
			case XBOX :
				return XboxController.Button.kRightBumper.value;
			default :
				return XboxController.Button.kRightBumper.value;
		}
	}
}
