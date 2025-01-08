package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

//VisionPortal
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar; 

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous (name = "VisionPortal", group = "Testing")
public class Vision extends LinearOpMode {
    private int closestX;
    private int closestY;
    @Override
    public void runOpMode() {
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()  //builds locator thingie
                .setTargetColorRange(ColorRange.BLUE) //insert proper colour values
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .build();
        
    
        VisionPortal portal = new VisionPortal.Builder() //makes the VisionPortal
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) //insert camera name
                .build();

        telemetry.setMsTransmissionInterval(50); //how often telemetry sends  data
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);  //type of screen?
        telemetry.addData("preview on/off", "... Camera Stream\n");

        closestY = 240;
        closestX = 320;

        while (opModeIsActive() || opModeInInit())
        {
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs(); //lists all of blobss
            
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs); //filter
        
            for(ColorBlobLocatorProcessor.Blob b : blobs) {  //displays all blobs
                RotatedRect boxFit = b.getBoxFit();
                /*telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));*/
                if ((boxFit.center.y < closestY)
                    || (boxFit.center.y == closestY && boxFit.center.x < closestX)) {
                    closestX = (int) boxFit.center.x;
                    closestY = (int) boxFit.center.y;

                }
            }
            if (closestX > 160) {
                moveRobot(1, 0, 0);
            } else if (closestX < 160) {
                moveRobot(-1, 0, 0);
            }

            telemetry.update();
        }
    }

    private void moveRobot(float x, float y, float rotX) {
        return;
    }
}
    