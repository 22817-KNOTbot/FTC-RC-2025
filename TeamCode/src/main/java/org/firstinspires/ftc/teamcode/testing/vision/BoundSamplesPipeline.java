package org.firstinspires.ftc.teamcode.testing.vision;

import org.opencv.core.CvType;
import org.opencv.core.Scalar;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Point;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BoundSamplesPipeline extends OpenCvPipeline {
	public static Scalar lowerYCrCb = new Scalar(0.0, 145.0, 145.0, 0.0);
	public static Scalar upperYCrCb = new Scalar(255.0, 160.0, 160.0, 0.0);
	private Mat ycrcbMat = new Mat();
	private Mat ycrcbBinaryMat = new Mat();

	private ArrayList<MatOfPoint> contours = new ArrayList<>();
	private Mat hierarchy = new Mat();

	private MatOfPoint2f contours2f = new MatOfPoint2f();
	private ArrayList<RotatedRect> contoursRotRects = new ArrayList<>();

	public Scalar lineColor = new Scalar(0.0, 0.0, 255.0, 0.0);
	public int lineThickness = 3;

	private Mat inputRotRects = new Mat();

	public Scalar lineColor1 = new Scalar(255.0, 0.0, 0.0, 0.0);
	public int lineThickness1 = 3;

	public static Scalar lowerYCrCb1 = new Scalar(0.0, 148.0, 63.0, 0.0);
	public static Scalar upperYCrCb1 = new Scalar(255.0, 255.0, 173.0, 0.0);
	private Mat ycrcbMat1 = new Mat();
	private Mat ycrcbBinaryMat1 = new Mat();

	private ArrayList<MatOfPoint> contours1 = new ArrayList<>();
	private Mat hierarchy1 = new Mat();

	private MatOfPoint2f contours12f = new MatOfPoint2f();
	private ArrayList<RotatedRect> contours1RotRects = new ArrayList<>();

	private Mat inputRotRectsRotRects = new Mat();

	public Scalar lineColor2 = new Scalar(255.0, 255.0, 0.0, 0.0);
	public int lineThickness2 = 3;

	public static Scalar lowerYCrCb2 = new Scalar(0.0, 116.0, 0.0, 0.0);
	public static Scalar upperYCrCb2 = new Scalar(255.0, 164.0, 57.0, 0.0);
	private Mat ycrcbMat2 = new Mat();
	private Mat ycrcbBinaryMat2 = new Mat();

	private ArrayList<MatOfPoint> contours2 = new ArrayList<>();
	private Mat hierarchy2 = new Mat();

	public int minArea = 0;
	public int maxArea = 1000;
	private ArrayList<MatOfPoint> contours2ByArea = new ArrayList<>();

	private MatOfPoint2f contours2ByArea2f = new MatOfPoint2f();
	private ArrayList<RotatedRect> contours2ByAreaRotRects = new ArrayList<>();

	private Mat inputRotRectsRotRectsRotRects = new Mat();


	private Mat YCrCb = new Mat();
	private int avg_y;
	private int avg_cr;
	private int avg_cb;

	@Override
	public Mat processFrame(Mat input) {
		Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
		Core.inRange(ycrcbMat, lowerYCrCb, upperYCrCb, ycrcbBinaryMat);

		contours.clear();
		hierarchy.release();
		Imgproc.findContours(ycrcbBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		contoursRotRects.clear();
		for(MatOfPoint points : contours) {
			contours2f.release();
			points.convertTo(contours2f, CvType.CV_32F);

			contoursRotRects.add(Imgproc.minAreaRect(contours2f));
		}

		input.copyTo(inputRotRects);
		for(RotatedRect rect : contoursRotRects) {
			if(rect != null) {
				Point[] rectPoints = new Point[4];
				rect.points(rectPoints);
				MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

				Imgproc.polylines(inputRotRects, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);
			}
		}

		Imgproc.cvtColor(input, ycrcbMat1, Imgproc.COLOR_RGB2YCrCb);
		Core.inRange(ycrcbMat1, lowerYCrCb1, upperYCrCb1, ycrcbBinaryMat1);

		contours1.clear();
		hierarchy1.release();
		Imgproc.findContours(ycrcbBinaryMat1, contours1, hierarchy1, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		contours1RotRects.clear();
		for(MatOfPoint points : contours1) {
			contours12f.release();
			points.convertTo(contours12f, CvType.CV_32F);

			contours1RotRects.add(Imgproc.minAreaRect(contours12f));
		}

		inputRotRects.copyTo(inputRotRectsRotRects);
		for(RotatedRect rect : contours1RotRects) {
			if(rect != null) {
				Point[] rectPoints = new Point[4];
				rect.points(rectPoints);
				MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

				Imgproc.polylines(inputRotRectsRotRects, Collections.singletonList(matOfPoint), true, lineColor1, lineThickness1);
			}
		}

		Imgproc.cvtColor(input, ycrcbMat2, Imgproc.COLOR_RGB2YCrCb);
		Core.inRange(ycrcbMat2, lowerYCrCb2, upperYCrCb2, ycrcbBinaryMat2);

		contours2.clear();
		hierarchy2.release();
		Imgproc.findContours(ycrcbBinaryMat2, contours2, hierarchy2, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		contours2ByArea.clear();
		for(MatOfPoint contour : contours2) {
			double area = Imgproc.contourArea(contour);
			if((area >= minArea) && (area <= maxArea)) {
				contours2ByArea.add(contour);
			}
		}

		contours2ByAreaRotRects.clear();
		for(MatOfPoint points : contours2ByArea) {
			contours2ByArea2f.release();
			points.convertTo(contours2ByArea2f, CvType.CV_32F);

			contours2ByAreaRotRects.add(Imgproc.minAreaRect(contours2ByArea2f));
		}

		inputRotRectsRotRects.copyTo(inputRotRectsRotRectsRotRects);
		for(RotatedRect rect : contours2ByAreaRotRects) {
			if(rect != null) {
				Point[] rectPoints = new Point[4];
				rect.points(rectPoints);
				MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

				Imgproc.polylines(inputRotRectsRotRectsRotRects, Collections.singletonList(matOfPoint), true, lineColor2, lineThickness2);
			}
		}


		// Colour telemetry
		Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

		Imgproc.rectangle(
			inputRotRectsRotRectsRotRects, // Buffer to draw on
			new Rect(
				new Point(140, 110), // First point which defines the rectangle
				new Point(180, 130)), // Second point which defines the rectangle
			new Scalar(255, 255, 255), // The color the rectangle is drawn in
			2); // Thickness of the rectangle lines

		Mat targetRegion = YCrCb.submat(new Rect(new Point(140, 110), new Point(180, 130)));

		avg_y = (int) Core.mean(targetRegion).val[0];
		avg_cr = (int) Core.mean(targetRegion).val[1];
		avg_cb = (int) Core.mean(targetRegion).val[2];

		return inputRotRectsRotRectsRotRects;
	}

	public int get_y() {
		return avg_cr;
	}

	public int get_Cr() {
		return avg_cr;
	}

	public int get_Cb() {
		return avg_cb;
	}
}