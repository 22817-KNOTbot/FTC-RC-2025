package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.bylazar.configurables.annotations.Configurable;

import java.util.function.Function;

@Configurable
public class GamepadManager {
	public static boolean PHYSICAL_GAMEPAD_1_ENABLED = true;
	public static boolean PHYSICAL_GAMEPAD_2_ENABLED = true;
	public static boolean VIRTUAL_GAMEPAD_1_ENABLED = true;
	public static boolean VIRTUAL_GAMEPAD_2_ENABLED = true;
	private Gamepad physicalGamepad1;
	private Gamepad physicalGamepad2;
	private Function<Gamepad, Gamepad> updateFunction1;
	private Function<Gamepad, Gamepad> updateFunction2;

	private Gamepad outputGamepad1 = new Gamepad();
	private Gamepad outputGamepad2 = new Gamepad();

	public GamepadManager(Gamepad gamepad1, Gamepad gamepad2, Function<Gamepad, Gamepad> updateFunction1, Function<Gamepad, Gamepad> updateFunction2) {
		this.physicalGamepad1 = gamepad1;
		this.physicalGamepad2 = gamepad2;
		this.updateFunction1 = updateFunction1;
		this.updateFunction2 = updateFunction2;
	}

	public void updateGamepads() {
		Gamepad newGamepad1;

		if (VIRTUAL_GAMEPAD_1_ENABLED) {
			newGamepad1 = updateFunction1.apply(PHYSICAL_GAMEPAD_1_ENABLED ? physicalGamepad1 : new Gamepad());
		} else {
			newGamepad1 = physicalGamepad1;
		}

		Gamepad newGamepad2;

		if (VIRTUAL_GAMEPAD_2_ENABLED) {
			newGamepad2 = updateFunction1.apply(PHYSICAL_GAMEPAD_2_ENABLED ? physicalGamepad2 : new Gamepad());
		} else {
			newGamepad2 = physicalGamepad2;
		}

		// TODO: test copying to physicalGamepad
		outputGamepad1.copy(newGamepad1);
		outputGamepad2.copy(newGamepad2);
	}

	public Gamepad getGamepad1() {
		return outputGamepad1;
	}

	public Gamepad getGamepad2() {
		return outputGamepad2;
	}
}
