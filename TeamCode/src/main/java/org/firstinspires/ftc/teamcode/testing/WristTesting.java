package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
// @TeleOp(name="Intake testing", group="Debug")
public class WristTesting extends LinearOpMode {
	public static double HEADING = 0;
	private Intake intake;
	private ServoImplEx intakeWrist;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		intake = new Intake(hardwareMap, true);

		waitForStart();

		while (opModeIsActive()) {
			intake.setPosition(Intake.Positions.INTAKE);
			double wristTarget = intakePosition(gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.toRadians(HEADING));

			telemetry.addData("Target", wristTarget);
			telemetry.update();
		}
	}

	/*
	 * Copied and slightly modified from Automations
	 */
	public double intakePosition(double x, double y, double heading) {
		double wristTarget = 0;
		if (x != 0 && y != 0) {
			double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
			double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);
			// This formula clips the angle to the range -90deg to 90.
			// Values outside this range are clipped to their opposite (ex: 135deg becomes -45deg)
			double clippedAngle = ((Math.toDegrees(Math.atan2(rotatedX * (rotatedY / Math.abs(rotatedY)), Math.abs(rotatedY))) + 180) % 360) - 180;
			double roundedAngle = Math.round(clippedAngle / 15) * 15;
			wristTarget = roundedAngle * Intake.WRIST_VALUE_PER_DEG;
			
		} else if (x == 0) {
			wristTarget = 0;
		} else if (y == 0) {
			wristTarget = 90 * Intake.WRIST_VALUE_PER_DEG;
		}
		intake.setWristRotation(wristTarget);
		return wristTarget;
	}
}