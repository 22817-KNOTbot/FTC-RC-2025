package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@Config
// @TeleOp(name="Shooter testing", group="Debug")
public class ShooterTesting extends LinearOpMode {
	public static boolean enabled = false;
	public static double desiredVelocity = Shooter.defaultVelocity;
	public static Float power = null;

	private Shooter shooter;

	@Override
	public void runOpMode() {
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		shooter = new Shooter(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
			shooter.updateVelocityPid();
			if (power == null) {
				shooter.desiredVelocity = desiredVelocity;
				shooter.targetVelocity = desiredVelocity + Shooter.velocityTargetOffset;
				shooter.enable(enabled);
			} else {
				shooter.setPower(power);
			}

			telemetry.addData("Enabled", enabled);
			telemetry.addData("Velocity", shooter.getVelocity());
			telemetry.update();
		}
	}
}