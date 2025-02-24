package org.firstinspires.ftc.teamcode.testing.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import java.util.List;

@Config
@Autonomous(name = "Auto Rotating Claw", group = "Testing")
public class AutoRotatingClaw extends LinearOpMode {
    private enum Sample_colour {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

	public static boolean LIVE_VIEW = true;
    public static Sample_colour SAMPLE_COLOUR = Sample_colour.BLUE;
    public static int ROI_X = 160;
    public static int ROI_Y = 120;
    
    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Scalar minScalar;
        Scalar maxScalar;

        switch(SAMPLE_COLOUR) {
            case RED:
                minScalar = new Scalar(0, 170, 65);
                maxScalar = new Scalar(255, 255, 134);
                break;
            case BLUE:
                minScalar = new Scalar(0, 0, 150);
                maxScalar = new Scalar(255, 255, 255);
                break;
            case YELLOW:
                minScalar = new Scalar(0, 0, 0);
                maxScalar = new Scalar(255, 255, 80);
                break;
            case NONE:
            default:
                minScalar = new Scalar(0, 0, 0);
                maxScalar = new Scalar(255, 255, 255);
                break;
        }

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(
                    ColorSpace.YCrCb, 
                    minScalar, 
                    maxScalar
                ))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .setDilateSize(2)
                .build();

        ColorBlobLocatorProcessor.BlobFilter areaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 2500, 61440);
        colorLocator.addFilter(areaFilter);    

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(LIVE_VIEW)
                .build();

        waitForStart();

        while (opModeIsActive())
        {
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            if (blobs.size() <= 0) continue; 

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

			double angle = Math.toDegrees(Math.atan2(distX * (distY / Math.abs(distY)), Math.abs(distY)));
			telemetry.addData("Angle", angle);
            telemetry.update();
        }
    }
}