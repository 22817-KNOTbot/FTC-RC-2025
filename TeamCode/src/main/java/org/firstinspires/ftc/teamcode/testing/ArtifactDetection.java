package org.firstinspires.ftc.teamcode.testing;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.processors.MultipleColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.MorphOperationType;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.graphics.Color;
import android.util.Size;

@Disabled
public class ArtifactDetection extends LinearOpMode {
	private MultipleColorBlobLocatorProcessor colorLocator;
	// private ColorBlobLocatorProcessor colorLocator;

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

		colorLocator = new MultipleColorBlobLocatorProcessor(
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

		// colorLocator = new ColorBlobLocatorProcessor.Builder()
        //         .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
        //         .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        //         .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
        //         .setDrawContours(true)   // Show contours on the Stream Preview
        //         .setBoxFitColor(0)       // Disable the drawing of rectangles
        //         .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
        //         .setBlurSize(5)          // Smooth the transitions between different colors in image

        //         // the following options have been added to fill in perimeter holes.
        //         .setDilateSize(15)       // Expand blobs to fill any divots on the edges
        //         .setErodeSize(15)        // Shrink blobs back to original size
        //         .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

        //         .build();

		ColorBlobLocatorProcessor.BlobFilter areaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 2500, 307200);
        colorLocator.addFilter(areaFilter);

		VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

		waitForStart();

		while (opModeIsActive()) {
			sleep(100);
		}
	}
}
