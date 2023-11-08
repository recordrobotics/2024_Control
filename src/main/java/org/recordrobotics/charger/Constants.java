// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;

/**
 * Put constants here
 */
public final class Constants {
	public static final String COMMANDS_TAB = "commands";
	public static final String DATA_TAB = "data";

	/**
	 * Control ports (PC USB)
	 */
	public class Control {
		// LegacyControl
		public static final int LEGACY_GAMEPAD = 0;

		// DoubleControl
		public static final int DOUBLE_GAMEPAD_1 = 0;
		public static final int DOUBLE_GAMEPAD_2 = 1;

	}

	public class Swerve {
		public static final double WHEEL = 6;
		public static final double GEAR_RATIO = 6;
		public final static double MOD_WIDTH = 0.762;
		public final static double MOD_LENGTH = 0.762;
	}

}
