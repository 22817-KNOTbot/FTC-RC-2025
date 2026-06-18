package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.bylazar.configurables.annotations.Configurable;

import java.util.function.Supplier;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class GamepadManager {
	public static boolean VIRTUAL_GAMEPAD_1_ENABLED = false;
	public static boolean VIRTUAL_GAMEPAD_2_ENABLED = false;
	private Gamepad physicalGamepad1;
	private Gamepad physicalGamepad2;
	private Supplier<Gamepad> virtualGamepad1;
	private Supplier<Gamepad> virtualGamepad2;

	private Gamepad outputGamepad1 = new Gamepad();
	private Gamepad outputGamepad2 = new Gamepad();

	public GamepadManager(Gamepad gamepad1, Gamepad gamepad2, Supplier<Gamepad> virtualGamepad1, Supplier<Gamepad> virtualGamepad2) {
		this.physicalGamepad1 = gamepad1;
		this.physicalGamepad2 = gamepad2;
		this.virtualGamepad1 = virtualGamepad1;
		this.virtualGamepad2 = virtualGamepad2;
	}

	public void updateGamepads() {
		Gamepad newGamepad1 = !VIRTUAL_GAMEPAD_1_ENABLED ? physicalGamepad1 : virtualGamepad1.get();

		Gamepad newGamepad2 = !VIRTUAL_GAMEPAD_2_ENABLED ? physicalGamepad2 : virtualGamepad2.get();

		outputGamepad1 = newGamepad1;
		outputGamepad2 = newGamepad2;
	}

	public Gamepad getGamepad1() {
		return outputGamepad1;
	}

	public Gamepad getGamepad2() {
		return outputGamepad2;
	}
}
