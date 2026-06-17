package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.FtcDashboard;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

// Robot Centric
@Disabled
public class BasicDriveTesting extends LinearOpMode {
	private GamepadManager gamepadManager;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		gamepadManager = new GamepadManager(gamepad1, gamepad2,
			PanelsGamepad.INSTANCE.getFirstManager()::getAsFTCGamepad,
			PanelsGamepad.INSTANCE.getSecondManager()::getAsFTCGamepad);
		gamepadManager.updateGamepads();
		Gamepad customGamepad1 = gamepadManager.getGamepad1();
		Gamepad customGamepad2 = gamepadManager.getGamepad2();

		DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
		DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
		DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
		DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

		frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
		backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
		frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
		backRightMotor.setDirection(DcMotor.Direction.FORWARD);
		frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		waitForStart();

		while (opModeIsActive()) {
			double xMovement = customGamepad1.left_stick_x;
			double yMovement = -customGamepad1.left_stick_y;
			double xRotation = customGamepad1.right_stick_x;

			double denominator = Math.max(Math.abs(xMovement) + Math.abs(yMovement) + Math.abs(xRotation), 1);
			double frontLeftPower = (yMovement + xMovement + xRotation) / denominator;
			double backLeftPower = (yMovement - xMovement + xRotation) / denominator;
			double frontRightPower = (yMovement - xMovement - xRotation) / denominator;
			double backRightPower = (yMovement + xMovement - xRotation) / denominator;

			frontLeftMotor.setPower(frontLeftPower);
			backLeftMotor.setPower(backLeftPower);
			frontRightMotor.setPower(frontRightPower);
			backRightMotor.setPower(backRightPower);

			telemetry.addData("frontLeftPower", frontLeftPower);
			telemetry.addData("backLeftPower", backLeftPower);
			telemetry.addData("frontRightPower", frontRightPower);
			telemetry.addData("backRightPower", backRightPower);
		}
	}
}
