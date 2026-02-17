package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

@Configurable
@Config
// @TeleOp(name="Transfer Testing", group="Debug")
public class TransferTesting extends LinearOpMode {
	public static double DESIRED_VELOCITY = 0;

	public static boolean START = false;
	public static boolean IGNORE_VELOCITY = false;

	private boolean shooting = false;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		Intake intake = new Intake(hardwareMap);
		Transfer transfer = new Transfer(hardwareMap);
		Shooter shooter = new Shooter(hardwareMap);
		
		waitForStart();

		shooter.enable(true);
		
		while (opModeIsActive()) {
			shooter.updateVelocityPid();
			shooter.desiredVelocity = DESIRED_VELOCITY;

			if (shooting) {
				transfer.transferUpdate();
				if (transfer.isFinishedTransferring()) {
					intake.enable(false);
					transfer.enable(false);
					shooting = false;
				}
			}

			if (START){
				intake.enable(true);
				transfer.enable(true);
				shooting = true;
				START = false;
			}

			telemetryManager.addData("Start", START);
			telemetryManager.addData("Shooting", shooting);
			telemetryManager.update();
		}
	}
}
