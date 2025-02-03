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
	public static double VALUE_PER_DEG = 0.0027777778;
	public static double HEADING = 0;
	private Intake intake;
	private ServoImplEx intakeWrist;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		// intake = new Intake(hardwareMap, true);

		intakeWrist = hardwareMap.get(ServoImplEx.class, "intakeWrist");
		// intakeWrist.setDirection(Servo.Direction.REVERSE);
		intakeWrist.setPwmRange(new ServoImplEx.PwmRange(500, 2500));
		intakeWrist.scaleRange(0.4, 0.6);

		waitForStart();

		while (opModeIsActive()) {
			intakePosition(telemetry, gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.toRadians(HEADING));
			telemetry.update();
		}
	}

	/*
	 * Copied and slightly modified from Automations
	 */
	public void intakePosition(Telemetry telemetry, double x, double y, double heading) {
		if (x != 0 && y != 0) {
			double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
			double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);
			// This formula clips the angle to the range -90deg to 90.
			// Values outside this range are clipped to their opposite (ex: 135deg becomes -45deg)
			double clippedAngle = ((Math.toDegrees(Math.atan2(rotatedX * (rotatedY / Math.abs(rotatedY)), Math.abs(rotatedY))) + 180) % 360) - 180;
			double roundedAngle = Math.round(clippedAngle / 15) * 15;
			double wristTarget = Intake.WRIST_MIDDLE_POSITION - (roundedAngle * (VALUE_PER_DEG)); // The number is servo position value per degree
			// intake.setWristRotation(wristTarget);
			intakeWrist.setPosition(wristTarget);

			telemetry.addData("clippedAngle", clippedAngle);
			telemetry.addData("wristTarget", wristTarget);
		} else if (x == 0) {
			intake.setWristRotation(Intake.WRIST_MIDDLE_POSITION);
			telemetry.addData("wristTarget", Intake.WRIST_MIDDLE_POSITION);
		} else if (y == 0) {
			intake.setWristRotation(Intake.WRIST_MIDDLE_POSITION + (90 * 0.0027777778));
			telemetry.addData("wristTarget", Intake.WRIST_MIDDLE_POSITION + (90 * 0.0027777778));
		}
	}
}