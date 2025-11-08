package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class MotifDecodeTesting extends LinearOpMode {
	@Override
	public void runOpMode() {
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		boolean previousDebug = Vision.DEBUG;
		Vision.DEBUG = true;

		Vision visionProcessor = new Vision(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
			Pattern newMotifPattern = visionProcessor.updateMotifPattern();
			Pattern storedMotifPattern = visionProcessor.getLastMotifPattern();
			telemetry.addLine(newMotifPattern != null ? newMotifPattern.toString() : "null");
			telemetry.addLine(newMotifPattern != null ? newMotifPattern.getPattern()[0].toString() : "null");
			telemetry.addLine(newMotifPattern != null ? newMotifPattern.getPattern()[1].toString() : "null");
			telemetry.addLine(newMotifPattern != null ? newMotifPattern.getPattern()[2].toString() : "null");
			telemetry.addLine("================================");
			telemetry.addLine(storedMotifPattern != null ? storedMotifPattern.toString() : "null");
			telemetry.addLine(storedMotifPattern != null ? storedMotifPattern.getPattern()[0].toString() : "null");
			telemetry.addLine(storedMotifPattern != null ? storedMotifPattern.getPattern()[1].toString() : "null");
			telemetry.addLine(storedMotifPattern != null ? storedMotifPattern.getPattern()[2].toString() : "null");
			visionProcessor.showTelemetry(telemetry);
			telemetry.update();
		}

		visionProcessor.close();
		Vision.DEBUG = previousDebug;
	}
}