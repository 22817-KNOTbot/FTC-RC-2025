package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

@Configurable
@Config
// @TeleOp(name="Transfer Testing", group="Debug")
public class TransferTesting extends LinearOpMode {
	public static double DESIRED_VELOCITY = 0;

	public static int ARTIFACT_PATTERN = 0;

	public static boolean START = false;
	public static boolean IGNORE_VELOCITY;


	private Artifact.Colour[] pattern;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		Storage storage = new Storage(hardwareMap, true);
		Shooter shooter = new Shooter(hardwareMap);

		shooter.enable(true);

		waitForStart();

		while (opModeIsActive()) {
			// storageMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
			// 		new PIDCoefficients(SPINDEXER_PID_P, SPINDEXER_PID_I, SPINDEXER_PID_D));

			shooter.updateVelocityPid();
			shooter.desiredVelocity = DESIRED_VELOCITY;

			storage.updateStorageArtifacts();
			storage.transferUpdate(IGNORE_VELOCITY || shooter.atDesiredVelocity());

			if (START){
				// switch (ARTIFACT_PATTERN) {
				// 	case 0:
				// 		pattern = Artifact.Pattern.GPP.getPattern();
				// 		break;
				// 	case 1:
				// 		pattern = Artifact.Pattern.PGP.getPattern();
				// 		break;
				// 	case 2:
				// 		pattern = Artifact.Pattern.PPG.getPattern();
				// 		break;
				// }
				// storage.turnToArtifactSequence(pattern);
				storage.transferStart(true);
				START = false;
			}

			telemetryManager.addData("Start", START);
			telemetryManager.addData("Transfer State", storage.getTransferState());
			telemetryManager.addData("Motor busy", storage.isMotorBusy());
			telemetryManager.update();
		}
	}
}
