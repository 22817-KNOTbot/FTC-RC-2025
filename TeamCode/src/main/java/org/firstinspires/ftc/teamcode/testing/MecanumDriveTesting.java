package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.FtcDashboard;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

public class MecanumDriveTesting extends LinearOpMode {
	private GamepadManager gamepadManager;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		gamepadManager = new GamepadManager(gamepad1, gamepad2,
				PanelsGamepad.INSTANCE.getFirstManager()::asCombinedFTCGamepad,
				PanelsGamepad.INSTANCE.getSecondManager()::asCombinedFTCGamepad);
		gamepadManager.updateGamepads();
		gamepad1.copy(gamepadManager.getGamepad1());
		gamepad2.copy(gamepadManager.getGamepad2());

		MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap);

		waitForStart();

		mecanumDrive.initialize();

		while (opModeIsActive()) {
			mecanumDrive.move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

			mecanumDrive.lockingMecanum(gamepad1.left_bumper);

			if (gamepad1.start || gamepad2.start) {
				mecanumDrive.resetPose();
			}

			Pose pose = mecanumDrive.getPose();
			Pose holdPose = mecanumDrive.getHoldPose();

			telemetryManager.addData("Position X", pose.getX());
			telemetryManager.addData("Position Y", pose.getY());

			telemetryManager.addData("Hold Position X", holdPose.getX());
			telemetryManager.addData("Hold Position Y", holdPose.getY());
		}
	}
}
