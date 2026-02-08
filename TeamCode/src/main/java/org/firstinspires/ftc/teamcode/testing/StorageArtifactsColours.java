package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StorageArtifactsColours extends LinearOpMode {
	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		Storage storage = new Storage(hardwareMap, true);

		waitForStart();

		while (opModeIsActive()) {
			storage.updateStorageArtifacts();

			telemetryManager.addData("Active", Storage.getActiveArtifact());
			telemetryManager.addData("Back left", Storage.getBackLeftArtifact());
			telemetryManager.addData("back right", Storage.getBackRightArtifact());
			telemetryManager.update();
		}
	}
}
