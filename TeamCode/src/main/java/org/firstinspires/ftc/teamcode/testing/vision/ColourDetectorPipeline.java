package org.firstinspires.ftc.teamcode.testing;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class ColourDetectorPipeline extends OpenCvPipeline {
	private final Point TARGET_TOP_LEFT     = new Point(140, 110);
	private final Point TARGET_BOTTOM_RIGHT = new Point(180, 130);
	private int sample_x;
	private int sample_y;
	private int sample_z;
	private Mat YCrCb = new Mat();
	private int avg_y;
	private int avg_cr;
	private int avg_cb;

	// @Override
	// public void init(Mat firstFrame) {
	// 	Imgproc.cvtColor(firstFrame, YCrCb, Imgproc.COLOR_RGB2YCrCb);
	// 	Core.extractChannel(YCrCb, Cr, 2);
	// 	Core.extractChannel(YCrCb, Cb, 2);
	// }

	@Override
    public Mat processFrame(Mat input) {
		Imgproc.rectangle(
			input, // Buffer to draw on
			TARGET_TOP_LEFT, // First point which defines the rectangle
			TARGET_BOTTOM_RIGHT, // Second point which defines the rectangle
			new Scalar(0, 0, 255), // The color the rectangle is drawn in
			2); // Thickness of the rectangle lines

		Mat targetRegion = input.submat(new Rect(TARGET_TOP_LEFT, TARGET_BOTTOM_RIGHT));

		Imgproc.cvtColor(targetRegion, YCrCb, Imgproc.COLOR_RGB2YCrCb);
		// Core.extractChannel(YCrCb, Cr, 2);
		// Core.extractChannel(YCrCb, Cb, 2);

		avg_y = (int) Core.mean(YCrCb).val[0];
		avg_cr = (int) Core.mean(YCrCb).val[1];
		avg_cb = (int) Core.mean(YCrCb).val[2];

		return input;
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