package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

@Configurable
@Config
// @TeleOp(name="Shooter velocity testing", group="Debug")
public class ShooterVelocityTesting extends LinearOpMode {
	public static double desiredVelocity = 0;

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
			shooter.desiredVelocity = desiredVelocity;

			telemetry.addData("Desired", shooter.desiredVelocity);
			telemetry.addData("Velocity", shooter.getVelocity());
			telemetry.update();
		}
	}
}