package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoAlign.AlignmentDirection;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class AutoAlignTesting extends LinearOpMode {
	public static Integer aprilTagId = null;

	@Override
	public void runOpMode() {
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		boolean previousDebug = Vision.DEBUG;
		Vision.DEBUG = true;

		Vision visionProcessor = new Vision(hardwareMap, aprilTagId);

		waitForStart();

		while (opModeIsActive()) {
			telemetry.addLine(visionProcessor.getAlignmentDirection().toString());
			visionProcessor.showTelemetry(telemetry);
			telemetry.update();
		}

		visionProcessor.close();
		Vision.DEBUG = previousDebug;
	}
}
