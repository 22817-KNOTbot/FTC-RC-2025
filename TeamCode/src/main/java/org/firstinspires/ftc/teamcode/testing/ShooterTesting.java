package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

@Configurable
@Config
// @TeleOp(name="Shooter testing", group="Debug")
@Disabled
public class ShooterTesting extends LinearOpMode {
	public static Pose robotPose = new Pose(72, 72, 0);
	public static Vector robotVelocity = new Vector(0, 0);
	public static Pose targetPose = new Pose(144, 144, 0);

	private Shooter shooter;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		shooter = new Shooter(hardwareMap);

		waitForStart();

		shooter.enable(true);

		while (opModeIsActive()) {
			shooter.updateVelocityPid();
			shooter.updateShooterTarget(robotPose, targetPose, robotVelocity);

			telemetry.addData("Desired", shooter.desiredVelocity);
			telemetry.addData("Velocity", shooter.getVelocity());
			telemetry.update();
		}
	}
}