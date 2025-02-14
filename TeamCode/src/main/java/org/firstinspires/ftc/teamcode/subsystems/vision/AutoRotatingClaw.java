package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.vision.MultipleColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

public class AutoRotatingClaw {
	private MultipleColorBlobLocatorProcessor colorLocator;
	private double angle = 0;
	private int ROI_X = 160;
    private int ROI_Y = 120;
    
	public AutoRotatingClaw(HardwareMap hardwareMap) {
		this(hardwareMap, false);
	}

	public AutoRotatingClaw(HardwareMap hardwareMap, boolean DEBUG) {
		List<List<Scalar>> colorRangeList = Arrays.asList(
			Arrays.asList(
				new Scalar(0, 170, 65),
				new Scalar(255, 255, 134)
			),
			Arrays.asList(
				new Scalar(0, 0, 150),
				new Scalar(255, 255, 255)
			),
			Arrays.asList(
				new Scalar(0, 0, 0),
				new Scalar(255, 255, 80)
			)
		);

        colorLocator = new MultipleColorBlobLocatorProcessor(
			colorRangeList,
			ImageRegion.entireFrame(),
			ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
			-1,
			2,
			DEBUG,
			5,
			Color.rgb(255, 120, 31),
			Color.rgb(255, 255, 255),
			Color.rgb(3, 227, 252)
		);

        ColorBlobLocatorProcessor.BlobFilter areaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 2500, 61440);
        colorLocator.addFilter(areaFilter);    

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(DEBUG)
                .build();
	}

	public void process() {
		List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
		if (blobs.size() <= 0) return; 

		RotatedRect closestRect = null;
		double closestRectDist = Integer.MAX_VALUE;
		for (ColorBlobLocatorProcessor.Blob blob : blobs) {
			RotatedRect rect = blob.getBoxFit();
			double distX = rect.center.x - ROI_X;
			double distY = rect.center.y - ROI_Y;

			double dist = Math.hypot(distX, distY);
			if (dist < closestRectDist) {
				closestRect = rect;
				closestRectDist = dist;
			}
		}

		Point[] points = new Point[4];
		closestRect.points(points);

		double bottomX = points[0].x;
		double bottomY = points[0].y;

		double dist1 = Math.hypot(points[1].x - bottomX, bottomY - points[1].y);
		double dist3 = Math.hypot(points[3].x - bottomX, bottomY - points[3].y);

		double distX;
		double distY;
		if (dist1 > dist3) {
			distX = points[1].x - bottomX;
			distY = bottomY - points[1].y;
		} else {
			distX = points[3].x - bottomX;
			distY = bottomY - points[3].y;
		}

		double angle;
		if (distX == 0) {
			angle = 0;
		} else if (distY == 0) {
			angle = 90;
		} else {
			angle = Math.toDegrees(Math.atan2(distX * (distY / Math.abs(distY)), Math.abs(distY)));
		}
		this.angle = angle;
	}

	public double getRotation() {
		return angle;
	}
}