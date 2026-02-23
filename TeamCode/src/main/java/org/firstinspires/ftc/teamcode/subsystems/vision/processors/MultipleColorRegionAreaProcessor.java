package org.firstinspires.ftc.teamcode.subsystems.vision.processors;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
public class MultipleColorRegionAreaProcessor implements VisionProcessor {
	private List<List<Scalar>> colorRangeList;
	private List<ImageRegion> roiRegions;
	private List<Rect> rois;
	private int frameWidth;
	private int frameHeight;
	private List<Mat> roiMats;
	private List<Mat> roiMats_userColorSpace;

	private Canvas maskBufferCanvas;
	private Bitmap maskBufferBitmap;

	private volatile List<Integer> userAreas = new ArrayList<>();

	public MultipleColorRegionAreaProcessor(List<List<Scalar>> colorRangeList, List<ImageRegion> rois) {
		this.colorRangeList = colorRangeList;
		this.roiRegions = rois;
		this.rois = new ArrayList<>();
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {
		frameWidth = width;
		frameHeight = height;

		for (ImageRegion roi : roiRegions) {
			try {
				Method openCvRectMethod = ImageRegion.class.getDeclaredMethod("asOpenCvRect", new Class[] {Integer.class, Integer.class});
				openCvRectMethod.setAccessible(true);
				this.rois.add((Rect) openCvRectMethod.invoke(roi, width, height));
				roiMats.add(null);
				roiMats_userColorSpace.add(null);
			} catch (Exception e) {
				throw new RuntimeException("Failed to convert ImageRegion to OpenCV Rect using reflection", e);
			}
		}

		maskBufferBitmap = Bitmap.createBitmap(frameWidth, frameHeight, Bitmap.Config.ARGB_8888);
		maskBufferCanvas = new Canvas(maskBufferBitmap);
	}

	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		List<Integer> areas = new ArrayList<>(rois.size());

		for (int i = 0; i < rois.size(); i++) {
			Rect roi = rois.get(i);
			if (roiMats.get(i) == null) {
				roiMats.set(i, frame.submat(roi));
				roiMats_userColorSpace.set(i, roiMats.get(i).clone());
			}

			Mat roiMat = roiMats.get(i);
			Mat roiMat_userColorSpace = roiMats_userColorSpace.get(i);

			Imgproc.cvtColor(roiMat, roiMat_userColorSpace, Imgproc.COLOR_RGB2YCrCb);

			areas.set(i, 0);
	
			for (List<Scalar> colorRange : colorRangeList) {
				Mat currentMat = roiMat_userColorSpace.clone();
				Mat currentMask = new Mat();
	
				Core.inRange(currentMat, colorRange.get(0), colorRange.get(1), currentMask);
	
				areas.set(i, areas.get(i) + Core.countNonZero(currentMask));
			
				currentMat.release();
				currentMask.release();
			}
	
			// Deep copy this to prevent concurrent modification exception
			userAreas = new ArrayList<>(areas);
		}

		return areas;
	}

	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
			float scaleCanvasDensity, Object userContext) {
		@SuppressWarnings("unchecked")
		ArrayList<Integer> areas = (ArrayList<Integer>) userContext;

		// canvas.drawBitmap(maskBufferBitmap, 0, 0, null);
		// TODO: Draw frame. For now it is fine without it
	}

	public List<Integer> getAreas() {
		return userAreas;
	}
}
