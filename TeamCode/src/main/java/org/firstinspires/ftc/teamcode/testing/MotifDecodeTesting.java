package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

@Configurable
@Config
public class MotifDecodeTesting extends LinearOpMode {
	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		boolean previousDebug = Vision.DEBUG;
		Vision.DEBUG = true;

		Vision visionProcessor = new Vision(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
			Pattern newMotifPattern = visionProcessor.updateMotifPattern();
			Pattern storedMotifPattern = visionProcessor.getLastMotifPattern();
			telemetryManager.addLine(newMotifPattern != null ? newMotifPattern.toString() : "null");
			telemetryManager.addLine(newMotifPattern != null ? newMotifPattern.getPattern()[0].toString() : "null");
			telemetryManager.addLine(newMotifPattern != null ? newMotifPattern.getPattern()[1].toString() : "null");
			telemetryManager.addLine(newMotifPattern != null ? newMotifPattern.getPattern()[2].toString() : "null");
			telemetryManager.addLine("================================");
			telemetryManager.addLine(storedMotifPattern != null ? storedMotifPattern.toString() : "null");
			telemetryManager.addLine(storedMotifPattern != null ? storedMotifPattern.getPattern()[0].toString() : "null");
			telemetryManager.addLine(storedMotifPattern != null ? storedMotifPattern.getPattern()[1].toString() : "null");
			telemetryManager.addLine(storedMotifPattern != null ? storedMotifPattern.getPattern()[2].toString() : "null");
			visionProcessor.showTelemetry(telemetryManager);
			telemetryManager.update();
		}

		visionProcessor.close();
		Vision.DEBUG = previousDebug;
	}
}