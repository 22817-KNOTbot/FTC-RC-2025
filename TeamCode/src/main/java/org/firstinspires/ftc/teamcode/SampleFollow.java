package org.firstinspires.ftc.teamcode;

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
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import com.acmerobotics.dashboard.config.Config;

import java.util.List;

@Config
@Autonomous(name = "Sample follow", group = "Subprograms")
public class SampleFollow extends LinearOpMode
{
    private enum Sample_colour {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    public static Sample_colour SAMPLE_COLOUR = Sample_colour.BLUE;
    public static int TARGET_X = 160;
    public static double MAX_SPEED = 0.5; // Replacement for Kp
    public static double Ki = 0;
    public static double Kd = 0;

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    
    @Override
    public void runOpMode()
    {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

         /*        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
         *                                    The higher the number of pixels, the more blurred the image becomes.
         *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
         *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *                                    Erosion can grow holes inside regions, and also shrink objects.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
         *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
         *                                    object, such as when removing noise from an image.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         */

        // A new Scalar(0, 0, 0), new Scalar(255, 255, 255)
        // R new Scalar(0, 170, 65), new Scalar(255, 255, 134)
        // B new Scalar(0, 0, 150), new Scalar(255, 255, 255)
        // Y new Scalar(0, 0, 0), new Scalar(255, 255, 80)
        Scalar minScalar;
        Scalar maxScalar;

        switch(SAMPLE_COLOUR) {
            case NONE:
                minScalar = new Scalar(0, 0, 0);
                maxScalar = new Scalar(255, 255, 255);
                break;
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
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .setDilateSize(2)
                .build();

        ColorBlobLocatorProcessor.BlobFilter areaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 2500, 20000);
        colorLocator.addFilter(areaFilter);    

        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .build();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        waitForStart();

        while (opModeIsActive())
        {
            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
             *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
             *
             * Use any of the following filters.
             *
             * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
             *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
             *   The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             */

            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
             *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
             *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
             */

            telemetry.addLine(" Area Density Aspect  Center");

            // Display the size (area) and center location for each Blob.
            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            }

            if (blobs.size() > 0) {
                int targetArea;
                String sampleOrientation = "Unknown";
                ColorBlobLocatorProcessor.Blob largestBlob = blobs.get(0);
                RotatedRect largestRect = largestBlob.getBoxFit();
                double aspectRatio = largestBlob.getAspectRatio();

                // Orientation finder
                if (3 > aspectRatio && aspectRatio > 0.3) {
                    if (aspectRatio > 1.5) { // horizontal
                        targetArea = 8000;
                        sampleOrientation = "Horizontal";
                    } else if (1.3 > aspectRatio && aspectRatio > 0.75) { // square
                        targetArea = 3000;
                        sampleOrientation = "Square";
                    } else if (0.5 > aspectRatio) { // vertical
                        targetArea = 8000;
                        sampleOrientation = "Vertical";
                    } else {
                        targetArea = largestBlob.getContourArea();
                        sampleOrientation = "Unknown";
                    }
                } else {
                    targetArea = largestBlob.getContourArea();
                    sampleOrientation = "Unknown";
                }

                int distX = (int) largestRect.center.x - TARGET_X;
                double speedX = ((double) distX/160) * MAX_SPEED;
                
                int distY = largestBlob.getContourArea() - targetArea;
                double speedY = ((double) distY/targetArea) * MAX_SPEED;

                telemetry.addData("Target", "X: %.2f Y: %.2f", distX, distY);
                telemetry.addData("Speed", "X: %.2f Y: %.2f", speedX, speedY);
                telemetry.addData("Orientation", sampleOrientation);
                moveRobot(speedX, speedY, 0);
            } else {
                moveRobot(0, 0, 0);
            } 

            telemetry.update();
        }
    }

    public void moveRobot(double x, double y, double rotX) {
        // Calculate wheel powers.
        double leftFrontPower    =  y + x + rotX;
        double rightFrontPower   =  y - x - rotX;
        double leftBackPower     =  y - x + rotX;
        double rightBackPower    =  y + x - rotX;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.max(Math.max(
            Math.abs(leftFrontPower), 
            Math.abs(rightFrontPower)), 
            Math.abs(leftBackPower)), 
            Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}