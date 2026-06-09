package org.firstinspires.ftc.teamcode.testing;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.firstinspires.ftc.teamcode.BuildConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.FtcLoggingSession;
import org.psilynx.psikit.ftc.wrappers.MotorWrapper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import kotlin.Unit;

@Configurable
@Config
// @TeleOp(name="Shooter velocity testing", group="Debug")
public class ShooterVelocityTesting extends LinearOpMode {
	public static double desiredVelocity = 0;
	public static double hoodAngle = 52.5;
	public static boolean intakeTransferEnabled = false;
	public static boolean useTurret = false;
	public static double turretTarget = Turret.BASE_ROTATION;

	private Shooter shooter;
	private Intake intake;
	private Transfer transfer;
	private Turret turret;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		final FtcLoggingSession loggingSession = new FtcLoggingSession();
		FtcLogTuning.bulkOnlyLogging = false;
		FtcLogTuning.logMotorCurrent = true;
		FtcLogTuning.motorCurrentReadPeriodSec = 0.2;
		MotorWrapper.logProfile = MotorWrapper.LOG_PROFILE_FULL;

		loggingSession.start(this, 5900, "", true, "/sdcard/FIRST/PsiKit/", null, this, () -> {
			String dateString = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss").format(new Date());
			Logger.recordMetadata("Date", dateString);
			Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
			Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
			Logger.recordMetadata("GitDirty", String.valueOf(BuildConstants.DIRTY));
			Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);

			return Unit.INSTANCE;
		});

		shooter = new Shooter(hardwareMap);
		intake = new Intake(hardwareMap);
		transfer = new Transfer(hardwareMap);
		if (useTurret) {
			turret = new Turret(hardwareMap);
		}

		waitForStart();

		shooter.enable(true);

		while (opModeIsActive()) {
			Logger.periodicBeforeUser();
			loggingSession.logOncePerLoop(this);

			shooter.updateVelocityPid();
			shooter.desiredVelocity = desiredVelocity;
			shooter.setPitchAngle(hoodAngle);
			intake.enable(intakeTransferEnabled);
			transfer.enable(intakeTransferEnabled);
			if (turret != null) {
				turret.setRotation(turretTarget);
			}

			telemetryManager.addData("Desired", shooter.desiredVelocity);
			telemetryManager.addData("Velocity", shooter.getVelocity());
			Logger.recordOutput("Desired", shooter.desiredVelocity);
			Logger.recordOutput("Velocity", shooter.getVelocity());
			Logger.recordOutput("Power", shooter.getPower()	);
			telemetryManager.update();

			Logger.periodicAfterUser(0.0, 0.0);
			idle();
		}

		loggingSession.end();
	}
}