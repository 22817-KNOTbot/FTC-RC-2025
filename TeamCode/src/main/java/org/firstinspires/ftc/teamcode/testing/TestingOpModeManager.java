package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.util.Arrays;
import java.util.List;

public final class TestingOpModeManager {
	public static final String GROUP = "Testing";
	public static final boolean DISABLED = false;

	private TestingOpModeManager() {
	}

	private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
		return new OpModeMeta.Builder()
				.setName(cls.getSimpleName())
				.setGroup(GROUP)
				.setFlavor(OpModeMeta.Flavor.TELEOP)
				.build();
	}

	@OpModeRegistrar
	public static void register(OpModeManager manager) {
		if (DISABLED)
			return;

		List<Class<? extends OpMode>> opModes = Arrays.asList(
			AprilTagPoseTesting.class,
			AutoAlignTesting.class,
			ColourTesting.class,
			HubLedTesting.class,
			IntakeTesting.class,
			MotifDecodeTesting.class,
			MotorTesting.class,
			MotorTwoTesting.class,
			ServoTesting.class,
			ShooterTesting.class,
			StorageTesting.class,
			TurretTesting.class
		);

		for (Class<? extends OpMode> opMode : opModes) {
			manager.register(metaForClass(opMode), opMode);
		}

		if (FtcDashboard.getInstance() != null) {
			FtcDashboard.getInstance().withConfigRoot(configRoot -> {
				for (Class<?> c : opModes) {
					configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
				}
			});
		}
	}
}