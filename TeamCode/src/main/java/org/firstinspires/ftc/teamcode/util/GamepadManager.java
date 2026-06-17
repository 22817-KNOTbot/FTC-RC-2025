package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.bylazar.configurables.annotations.Configurable;

import java.util.function.Supplier;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class GamepadManager {
	public static boolean PHYSICAL_GAMEPAD_1_ENABLED = true;
	public static boolean PHYSICAL_GAMEPAD_2_ENABLED = true;
	public static boolean VIRTUAL_GAMEPAD_1_ENABLED = true;
	public static boolean VIRTUAL_GAMEPAD_2_ENABLED = true;
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
		Gamepad newGamepad1 = new Gamepad();

		if (PHYSICAL_GAMEPAD_1_ENABLED) {
			newGamepad1 = physicalGamepad1;
		}
		if (VIRTUAL_GAMEPAD_1_ENABLED) {
			copy(virtualGamepad1.get(), newGamepad1);
		}

		Gamepad newGamepad2 = new Gamepad();

		if (PHYSICAL_GAMEPAD_2_ENABLED) {
			newGamepad2 = physicalGamepad2;
		}
		if (VIRTUAL_GAMEPAD_2_ENABLED) {
			copy(virtualGamepad2.get(), newGamepad2);
		}

		copy(newGamepad1, outputGamepad1);
		copy(newGamepad2, outputGamepad2);
	}

	public Gamepad getGamepad1() {
		return outputGamepad1;
	}

	public Gamepad getGamepad2() {
		return outputGamepad2;
	}

	private void copy(Gamepad input, Gamepad output) {
		output.left_stick_x = input.left_stick_x != (double)0.0F ? (float)input.left_stick_x : output.left_stick_x;
		output.left_stick_y = input.left_stick_y != (double)0.0F ? (float)input.left_stick_y : output.left_stick_y;
		output.right_stick_x = input.right_stick_x != (double)0.0F ? (float)input.right_stick_x : output.right_stick_x;
		output.right_stick_y = input.right_stick_y != (double)0.0F ? (float)input.right_stick_y : output.right_stick_y;
		output.left_trigger = input.left_trigger != (double)0.0F ? (float)input.left_trigger : output.left_trigger;
		output.right_trigger = input.right_trigger != (double)0.0F ? (float)input.right_trigger : output.right_trigger;
		output.left_bumper = input.left_bumper || output.left_bumper;
		output.right_bumper = input.right_bumper || output.right_bumper;
		output.a = input.cross || output.a;
		output.b = input.circle || output.b;
		output.x = input.square || output.x;
		output.y = input.triangle || output.y;
		output.cross = output.a;
		output.circle = output.b;
		output.square = output.x;
		output.triangle = output.y;
		output.dpad_up = input.dpad_up || output.dpad_up;
		output.dpad_down = input.dpad_down || output.dpad_down;
		output.dpad_left = input.dpad_left || output.dpad_left;
		output.dpad_right = input.dpad_right || output.dpad_right;
		output.guide = input.ps || output.guide;
		output.ps = output.guide;
		output.options = input.options || output.options;
		output.back = input.share || output.back;
		output.share = output.back;
		output.touchpad = input.touchpad || output.touchpad;
		output.left_stick_button = input.left_stick_button || output.left_stick_button;
		output.right_stick_button = input.right_stick_button || output.right_stick_button;
	}
}
