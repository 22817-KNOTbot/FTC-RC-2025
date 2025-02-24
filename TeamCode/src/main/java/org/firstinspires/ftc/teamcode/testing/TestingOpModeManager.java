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
    public static final String GROUP = "Debug";
    public static final boolean DISABLED = true;

    private TestingOpModeManager() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

		List<Class> opModes = Arrays.asList(
			CameraSetting.class,
			ColourTesting.class,
			ConceptAprilTagOptimizeExposure.class,
			GamepadTesting.class,
			HardwareDeviceTesting.class,
			HubLedTesting.class,
			ManualSlideControl.class,
			MotorTesting.class,
			PwmTesting.class,
			SampleFollow.class,
			ServoTesting.class,
			SlideTesting.class,
			WristTesting.class,
			TouchSensorTesting.class
		);

		for (Class<? extends OpMode> opMode : opModes) {
			manager.register(metaForClass(opMode), opMode);
		}

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : opModes) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
