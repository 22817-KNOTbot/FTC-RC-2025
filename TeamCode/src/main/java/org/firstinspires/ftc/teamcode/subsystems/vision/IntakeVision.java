package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import android.graphics.Color;
import android.util.Size;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.Blob;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.MorphOperationType;
import org.opencv.core.Scalar;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.vision.processors.MultipleColorBlobLocatorProcessor;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class IntakeVision {
	public static boolean DEBUG = false;
	public static int decimation = 3;
	public static List<List<Scalar>> colorRangeList = Arrays.asList(
			// Purple
			Arrays.asList(
					new Scalar(0, 135 - 5, 135 - 0),
					new Scalar(255, 155 + 5, 169 + 20)),
			// Green
			Arrays.asList(
					new Scalar(0 - 0, 50 - 10, 118 - 25),
					new Scalar(255 - 10, 105 + 10, 145 + 15)));
	public static double minBlobArea = 2500;
	public static double maxBlobArea = 307200;

	private VisionPortal visionPortal;

	private MultipleColorBlobLocatorProcessor colorProcessor;

	private CameraStream cameraStreamProcessor;

	private WebcamName webcam;

	public IntakeVision(HardwareMap hardwareMap) {
		this(hardwareMap, null);
	}

	public IntakeVision(HardwareMap hardwareMap, Integer targetAprilTagId) {
		WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

		colorProcessor = new MultipleColorBlobLocatorProcessor(
			colorRangeList,
			ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY,
			MorphOperationType.OPENING,
			55,
			70,
			true,
			10,
			Color.rgb(255, 120, 31),
			Color.rgb(0, 255, 0),
			Color.rgb(255, 255, 255),
			Color.rgb(3, 227, 252)
		);

		ColorBlobLocatorProcessor.BlobFilter areaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 2500, 307200);
        colorProcessor.addFilter(areaFilter);

		cameraStreamProcessor = new CameraStream();

		VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder()
				.setCamera(webcam)
				.setCameraResolution(new Size(640, 480))
				.enableLiveView(DEBUG)
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.addProcessor(colorProcessor);
		
		if (DEBUG) {
			visionPortalBuilder.addProcessor(cameraStreamProcessor);
			FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 0);
		}
		visionPortal = visionPortalBuilder.build();
	}

	public List<Blob> getBlobs() {
		return colorProcessor.getBlobs();
	}

	public void setColorProcessorEnabled(boolean enabled) {
		visionPortal.setProcessorEnabled(colorProcessor, enabled);
	}

	public void setAllProcessorsEnabled(boolean enabled) {
		setColorProcessorEnabled(enabled);
	}

	public void close() {
		visionPortal.close();
		PanelsCameraStream.INSTANCE.stopStream();
	}

	public void showTelemetry(TelemetryManager telemetry) {
		// TODO: Add as needed
	}
}
