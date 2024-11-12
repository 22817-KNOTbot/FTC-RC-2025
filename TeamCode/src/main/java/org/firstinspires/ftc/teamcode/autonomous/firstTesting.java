package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamode.MecanumDrive;
// import org.firstinspires.ftc.teamode.AprilTagDrive;

@Config
@Autonomous(name = "First Test Auto", group = "Testing")
public class FirstTesting extends LinearOpMode {
    public static double initialPoseX = 0;
    public static double initialPoseY = 0;
    public static double initialPoseHeading = 90;
    
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(initialPoseX, initialPoseY, Math.toRadians(initialPoseHeading));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // AprilTagDrive drive = new AprilTagDrive(hardwareMap, initialPose, aprilTagProcessor, true);

        Action driveToSpecimenHang = drive.actionBuilder(initialPose)
            .splineToLinearHeading(new Pose2d(-7, -35, Math.toRadians(270)), Math.toRadians(90))
            .build();

        Action driveToSampleIntake1 = drive.actionBuilder(new Pose2d(-7, -35, Math.toRadians(270)))
            .splineToSplineHeading(new Pose2d(-30, -40, Math.toRadians(180)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-39, -25, Math.toRadians(180)), Math.toRadians(190))
            .build();

        Action driveToSampleDeposit1 = drive.actionBuilder(new Pose2d(-39, -25, Math.toRadians(180)))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
            .build();

        Action driveToSampleIntake2 = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
            .splineToSplineHeading(new Pose2d(-49, -25, Math.toRadians(180)), Math.toRadians(190))
            .build();

        Action driveToSampleDeposit2 = drive.actionBuilder(new Pose2d(-49, -25, Math.toRadians(180)))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
            .build();

        Action driveToSampleIntake3 = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
            .splineToSplineHeading(new Pose2d(-59, -25, Math.toRadians(180)), Math.toRadians(135))
            .build();

        Action driveToSampleDeposit3 = drive.actionBuilder(new Pose2d(-59, -25, Math.toRadians(180)))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
            .build();

        Action driveToAscend = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
            .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(0))
            .build();

        Actions.runBlocking(
            new SequentialAction(
                driveToSpecimenHang,
                placeholderAction(),
                driveToSampleIntake1,
                placeholderAction(),
                driveToSampleDeposit1,
                placeholderAction(),
                driveToSampleIntake2,
                placeholderAction(),
                driveToSampleDeposit2,
                placeholderAction(),
                driveToSampleIntake3,
                placeholderAction(),
                driveToSampleDeposit3,
                placeholderAction(),
                driveToAscend
            )
        );
    }

    // public class PlaceholderAction implements Action {
    //     private boolean initialized = false;

    //     @Override
    //     public boolean run(@NonNull TelemetryPacket packet) {
    //         if (!initialized) {
    //             initialized = true;
    //             return true;
    //         }

    //         // packet.put("Foo", "bar");

    //         return false;
    //     }
    // }

    public Action placeholderAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    return true;
                }
    
                // packet.put("Foo", "bar");
    
                return false;
            }
        };
    }
}
