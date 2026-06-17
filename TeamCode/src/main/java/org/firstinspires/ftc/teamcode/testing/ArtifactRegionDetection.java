package org.firstinspires.ftc.teamcode.testing;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.processors.MultipleColorRegionAreaProcessor;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.util.Size;

@Disabled
public class ArtifactRegionDetection extends LinearOpMode {
	private MultipleColorRegionAreaProcessor colorAreaProcessor;

	@Override
	public void runOpMode() {
		List<List<Scalar>> colorRangeList = Arrays.asList(
			// Purple
			Arrays.asList(
				new Scalar(  0, 135 -  5, 135 -  0),
            	new Scalar(255, 155 +  5, 169 + 20)
			),
			// Green
			Arrays.asList(
				new Scalar(  0 -  0,  50 - 10, 118 - 25),
            	new Scalar(255 - 10, 105 + 10, 145 + 15)
			)
		);

		List<ImageRegion> rois = Arrays.asList(
			ImageRegion.asUnityCenterCoordinates(-1, 1, 0, -1),
			ImageRegion.asUnityCenterCoordinates(0, 1, 1, -1)
		);

		colorAreaProcessor = new MultipleColorRegionAreaProcessor(
			colorRangeList,
			rois
		);

		VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorAreaProcessor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		// telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		waitForStart();

		while (opModeIsActive()) {
			List<Integer> areas = colorAreaProcessor.getAreas();
			for (int i = 0; i < areas.size(); i++) {
				telemetryManager.addData("Area " + i, areas.get(i));
			}
		}
	}
}
