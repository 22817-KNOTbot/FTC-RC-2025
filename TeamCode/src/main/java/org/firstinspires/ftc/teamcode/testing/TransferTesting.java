package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.scoring.Artifact;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

@Configurable
@Config
// @TeleOp(name="Transfer Testing", group="Debug")
public class TransferTesting extends LinearOpMode {
	public static final String MOTOR_NAME_SPINDEXER = "storageMotor";
	public static final String MOTOR_NAME_INTAKE = "intakeMotor";
	public static final String MOTOR_NAME_SHOOTER_1 = "shooterMotorLeft";
	public static final String MOTOR_NAME_SHOOTER_2 = "shooterMotorRight";
	
	public static double POWER_SPINDEXER = 0;
	public static double POWER_INTAKE = 0;
	public static double POWER_SHOOTER = 0;
	public static int SPINDEXER_TARGET = 0;
	public static boolean SPINDEXER_RESET_ENCODER = true;
	public static double TRANSFER_RIGHT_POSITION = 0.47;

	public static double SPINDEXER_PID_P = 10;
	public static double SPINDEXER_PID_I = 0;
	public static double SPINDEXER_PID_D = 0;

	public static int ARTIFACT_PATTERN = 0;

	public static boolean START = false;


	private Artifact.Colour[] pattern;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		Storage storage = new Storage(hardwareMap, true);
		Shooter shooter = new Shooter(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
			// storageMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
			// 		new PIDCoefficients(SPINDEXER_PID_P, SPINDEXER_PID_I, SPINDEXER_PID_D));

			shooter.enable(true);

			storage.updateStorageArtifacts();
			storage.transferUpdate(true);

			if (START){
				switch (ARTIFACT_PATTERN) {
					case 0:
						pattern = Artifact.Pattern.GPP.getPattern();
						break;
					case 1:
						pattern = Artifact.Pattern.PGP.getPattern();
						break;
					case 2:
						pattern = Artifact.Pattern.PPG.getPattern();
						break;
				}
				storage.turnToArtifactSequence(pattern);
				storage.transferStart();
				START = false;
			}
		}
	}
}
