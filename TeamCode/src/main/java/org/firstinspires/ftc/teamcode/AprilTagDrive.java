package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.sun.source.tree.LabeledStatementTree;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public class AprilTagDrive extends MecanumDrive {
    public static double APRIL_WEIGHT = 0.5;
    public static double POSE_WEIGHT = 0.5;
    private AprilTagProcessor aprilTag;
    private boolean firstTagFound;
    private long tagDetectTime;
    
    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTag, boolean estimateWithAprilTag) {
        super(hardwareMap, pose);
        this.aprilTag = aprilTag;
        // estimateWithAprilTag means to set pose from april tag on first detection. Otherwise initial pose is purely
        // based on constructor input. An estimate in the constructor is still required.
        this.firstTagFound = !estimateWithAprilTag;
    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        // RR standard: get the latest pose from the upstream updatePoseEstimate
        // that will change the pose variable to the pose based on odo or drive encoders (or OTOS)
        PoseVelocity2d posVel = super.updatePoseEstimate();
        // Get the absolute position from the camera
        Vector2d aprilVector = getVectorBasedOnTags();


        // it's possible we can't see any tags, so we need to check for a vector of 0
        if (aprilVector != null) {
            Vector2d processedVector;
            // (aprilVector*APRIL_WEIGHT+pose*POSE_WEIGHT)/(APRIL_WEIGHT+POSE_WEIGHT)
            if (firstTagFound) {
                processedVector = aprilVector.times(APRIL_WEIGHT).plus(pose.position.times(POSE_WEIGHT)).div(APRIL_WEIGHT+POSE_WEIGHT);
            } else {
                processedVector = aprilVector;
                firstTagFound = true;
            }
            pose = new Pose2d(processedVector, pose.heading);
        }

        return posVel; // trust the existing localizer for speeds because I don't know how to do it with apriltags
    }
    public Vector2d getVectorBasedOnTags() {
        ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            return null;
        } else {
            tagDetectTime = aprilTag.getDetections().get(0).frameAcquisitionNanoTime;
            return aprilTag.getDetections().stream() // get the tag detections as a Java stream
                    .map(detection -> {
                        double ftcX = detection.robotPose.getPosition().x / 24.8611111111;
                        double ftcY = detection.robotPose.getPosition().y / 24.8611111111;
                        double offset = pose.heading.log() - (detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI/2);
                        double x = ftcX*Math.cos(offset) + ftcY*Math.sin(offset);
                        double y = ftcX*-Math.sin(offset) + ftcY*Math.cos(offset);
                        return new Vector2d(x, y);
                    })
                    // add them together
                    .reduce(new Vector2d(0, 0), Vector2d::plus)
                    // divide by the number of tags to get the average position
                    .div(aprilTag.getDetections().size());
        }
    }
}