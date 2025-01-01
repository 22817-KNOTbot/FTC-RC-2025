package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Drawing;

@Config
@Disabled
@TeleOp(name="Gamepad Testing", group="Debug")
public class GamepadTesting extends LinearOpMode {
	public static int RUMBLE_DURATION = 500;
	public static int COLOUR_RED = 255;
	public static int COLOUR_GREEN = 255;
	public static int COLOUR_BLUE = 255;
	public static int COLOUR_DURATION = 1000;

	private FtcDashboard dashboard = FtcDashboard.getInstance();
	private boolean buttonPressed = false;
	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

		waitForStart();

		while(opModeIsActive()) {
			if (!buttonPressed) {
				if (gamepad1.ps) {
					gamepad1.rumble(RUMBLE_DURATION);
				}
				if (gamepad1.back && gamepad1.start) {
					gamepad1.rumble(1, 1, RUMBLE_DURATION);
				} else if (gamepad1.back) {
					gamepad1.rumble(1, 0, RUMBLE_DURATION);
				} else if (gamepad1.start) {
					gamepad1.rumble(0, 1, RUMBLE_DURATION);
				}
				
				if (gamepad1.a) {
					gamepad1.setLedColor(COLOUR_RED, COLOUR_GREEN, COLOUR_BLUE, COLOUR_DURATION);
				}
			}
			buttonPressed = !gamepad1.atRest();

			telemetry.addData("touchpad", gamepad1.touchpad);
			telemetry.addData("touchpad_finger_1", gamepad1.touchpad_finger_1);
			telemetry.addData("touchpad_finger_2", gamepad1.touchpad_finger_2);
			telemetry.addData("touchpad_finger_1_x", gamepad1.touchpad_finger_1_x);
			telemetry.addData("touchpad_finger_1_y", gamepad1.touchpad_finger_1_y);
			telemetry.addData("touchpad_finger_2_x", gamepad1.touchpad_finger_2_x);
			telemetry.addData("touchpad_finger_2_y", gamepad1.touchpad_finger_2_y);

			if (gamepad1.touchpad_finger_1) {
				TelemetryPacket packet = new TelemetryPacket();
				Canvas canvas = packet.fieldOverlay();
				Drawing.drawRobot(canvas, new Pose2d(gamepad1.touchpad_finger_1_x*72, gamepad1.touchpad_finger_1_y*72, 0));
				dashboard.sendTelemetryPacket(packet);
			}
			telemetry.update();

		}
	}
}
