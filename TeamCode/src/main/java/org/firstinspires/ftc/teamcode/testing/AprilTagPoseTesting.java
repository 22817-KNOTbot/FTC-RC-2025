package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.vision.CameraStream;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import java.util.List;

@Configurable
@Config
public class AprilTagPoseTesting extends LinearOpMode {
	public static double robotRadius = 9;
	public static Integer aprilTagId = null;
	public static Position cameraPosition = new Position(DistanceUnit.INCH,
			0, 0, 0, 0);
	public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
			0, -90, 0, 0);

	private VisionPortal visionPortal;
	private AprilTagProcessor aprilTagProcessor;

	private CameraStream cameraStreamProcessor;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

		aprilTagProcessor = new AprilTagProcessor.Builder()
				.setDrawAxes(true)
				.setDrawCubeProjection(true)
				.setCameraPose(cameraPosition, cameraOrientation)
				.build();

		aprilTagProcessor.setDecimation(Vision.decimation);

		cameraStreamProcessor = new CameraStream();

		visionPortal = new VisionPortal.Builder()
				.setCamera(webcam)
				.enableLiveView(true)
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.addProcessor(aprilTagProcessor)
				.addProcessor(cameraStreamProcessor)
				.build();

		PanelsCameraStream.INSTANCE.startStream(cameraStreamProcessor, null);
		FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 0);

		waitForStart();

		while (opModeIsActive()) {
			AprilTagDetection targetedAprilTag = null;

			List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
			for (AprilTagDetection detection : detections) {
				if (aprilTagId == null || aprilTagId < 0 || detection.id == aprilTagId) {
					targetedAprilTag = detection;
					break;
				}
			}

			/*
			 * Telemetry
			 */
			if (targetedAprilTag != null) {
				if (targetedAprilTag.metadata != null) {
					telemetryManager.addLine(String.format("\n==== (ID %d) %s", targetedAprilTag.id, targetedAprilTag.metadata.name));
					telemetryManager.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
							targetedAprilTag.robotPose.getPosition().x,
							targetedAprilTag.robotPose.getPosition().y,
							targetedAprilTag.robotPose.getPosition().z));
					telemetryManager.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
							targetedAprilTag.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
							targetedAprilTag.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
							targetedAprilTag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
				} else {
					telemetryManager.addLine(String.format("\n==== (ID %d) Unknown", targetedAprilTag.id));
				}
			} else {
				telemetryManager.addLine("\n==== (NONE)");
			}

			telemetryManager.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
			telemetryManager.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

			/*
			 * Dashboards drawing
			 */
			drawDashboardRobot(telemetryManager.getDashboardCanvas(), targetedAprilTag.robotPose);

			FieldManager fieldManager = PanelsField.INSTANCE.getField();
			drawPanelsRobot(fieldManager, targetedAprilTag.robotPose);

			fieldManager.update();
			telemetryManager.update();
		}

		visionPortal.close();
		PanelsCameraStream.INSTANCE.stopStream();
	}

	private void drawDashboardRobot(Canvas canvas, Pose3D pose) {
		Position position = pose.getPosition();
		YawPitchRollAngles orientation = pose.getOrientation();

		canvas.setStrokeWidth(1);
		canvas.strokeCircle(position.x, position.y, robotRadius);

		Position endPosition = new Position(DistanceUnit.INCH, Math.cos(Math.toRadians(orientation.getYaw(AngleUnit.DEGREES))) * robotRadius,
				Math.sin(Math.toRadians(orientation.getYaw(AngleUnit.DEGREES))) * robotRadius, 0, 0);
		canvas.strokeLine(position.x, position.y, endPosition.x, endPosition.y);
	}

	private void drawPanelsRobot(FieldManager fieldManager, Pose3D pose) {
		Position position = pose.getPosition();
		YawPitchRollAngles orientation = pose.getOrientation();

		fieldManager.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());
		fieldManager.setStyle("red", "white", 1);

		fieldManager.moveCursor(position.x, position.y);
		fieldManager.circle(robotRadius);

		Position endPosition = new Position(DistanceUnit.INCH, Math.cos(Math.toRadians(orientation.getYaw(AngleUnit.DEGREES))) * robotRadius,
				Math.sin(Math.toRadians(orientation.getYaw(AngleUnit.DEGREES))) * robotRadius, 0, 0);
		fieldManager.line(endPosition.x, endPosition.y);
	}
}
