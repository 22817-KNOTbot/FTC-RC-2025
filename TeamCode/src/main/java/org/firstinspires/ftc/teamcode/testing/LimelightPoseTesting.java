package org.firstinspires.ftc.teamcode.testing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLFieldMap.Fiducial;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

@Config
@Configurable
public class LimelightPoseTesting extends LinearOpMode {
	public static int PIPELINE = 1;
	public static int HEADING_DEG = 90; // 90 = obelisk, ccw positive

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
		// IMU imu = hardwareMap.get(IMU.class, "imu");
		// imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
		// 		RevHubOrientationOnRobot.LogoFacingDirection.UP,
		// 		RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
		// imu.resetYaw();
		limelight.setPollRateHz(100);
		limelight.start();

		limelight.pipelineSwitch(PIPELINE);

		List<Fiducial> fiducials = new ArrayList<>();
		fiducials.add(new Fiducial(24, 165.1, "apriltag3_36h11_classic", new ArrayList<Double>(Arrays.asList(0.5877852522924731d,0.8090169943749473d,0d,-1.4827d,-0.8090169943749473d,0.5877852522924731d,0d,1.4133d,0d,0d,1d,0.7493d,0d,0d,0d,1d)), true));
		// fiducials.add(new Fiducial(24, 165.1, "apriltag3_36h11_classic", new ArrayList<Double>(Arrays.asList(-0.3927376690073108, -0.9196505441431021d, 0d, 1.4827d, 0.9196505441431021d, -0.3927376690073108d, 0d, -1.4133d, 0d, 0d, 1d, 0.7493d, 0d, 0d, 0d, 1d)), true));
		LLFieldMap fieldMap = new LLFieldMap(fiducials, "ftc");
		boolean success = limelight.uploadFieldmap(fieldMap, null);

		Drawing.init();

		waitForStart();

		while (opModeIsActive()) {
			// double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
			// limelight.updateRobotOrientation(robotYaw);
			limelight.updateRobotOrientation(HEADING_DEG);
			
			LLResult result = limelight.getLatestResult();
			if (result != null && result.isValid()) {
				Pose3D botpose = result.getBotpose();
				Pose3D botpose_mt2 = result.getBotpose_MT2();
				if (botpose != null) {
					double x = botpose.getPosition().x / DistanceUnit.mPerInch;
					double y = botpose.getPosition().y / DistanceUnit.mPerInch;
					telemetryManager.addData("MT1 Location", "(" + x + ", " + y + ")");

					Pose pedroPose = new Pose(y + 72, -x + 72, botpose.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI / 2);
					Drawing.drawRobot(pedroPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE), new Style("", "#47b53fff", 0.75), telemetryManager.getDashboardCanvas());
				}
				if (botpose_mt2 != null) {
					double x = botpose_mt2.getPosition().x / DistanceUnit.mPerInch;
					double y = botpose_mt2.getPosition().y / DistanceUnit.mPerInch;
					telemetryManager.addData("MT2 Location:", "(" + x + ", " + y + ")");

					Pose pedroPose = new Pose(x + (72 - -100), y + (144 - 57), botpose_mt2.getOrientation().getYaw(AngleUnit.RADIANS));
					Drawing.drawRobot(pedroPose, new Style("", "#3F51B5", 0.75), telemetryManager.getDashboardCanvas());

					// Pose pedroPose = new Pose(x, y, botpose_mt2.getOrientation().getYaw(AngleUnit.RADIANS), InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
					// telemetryManager.addData("MT2 Converted location", pedroPose);
					// Drawing.drawRobot(pedroPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE), new Style("", "#3F51B5", 0.75), telemetryManager.getDashboardCanvas());
				}
			}

			telemetryManager.addData("Uploaded field", success);
			// telemetryManager.addData("Yaw", robotYaw);
			telemetryManager.addData("Yaw", HEADING_DEG);
			telemetryManager.addData("Pipeline", result != null ? result.getPipelineIndex() : "Unknown");
			telemetryManager.addData("Pipeline Type", result != null ? result.getPipelineType() : "Unknown");

			Drawing.sendPacket();
			telemetryManager.update();
		}
	}
}
