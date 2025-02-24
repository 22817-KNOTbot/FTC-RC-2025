package pedroPathing.tuners_tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import pedroPathing.tuners_tests.automatic.ForwardVelocityTuner;
import pedroPathing.tuners_tests.automatic.ForwardZeroPowerAccelerationTuner;
import pedroPathing.tuners_tests.automatic.LateralZeroPowerAccelerationTuner;
import pedroPathing.tuners_tests.automatic.StrafeVelocityTuner;
import pedroPathing.tuners_tests.localization.ForwardTuner;
import pedroPathing.tuners_tests.localization.LateralTuner;
import pedroPathing.tuners_tests.localization.LocalizationTest;
import pedroPathing.tuners_tests.localization.MotorDirections;
import pedroPathing.tuners_tests.localization.TurnTuner;
import pedroPathing.tuners_tests.pid.CurvedBackAndForth;
import pedroPathing.tuners_tests.pid.StraightBackAndForth;

import java.util.Arrays;
import java.util.List;

public final class TuningOpModeManager {
    public static final String GROUP = "Pedro Tuning";
    public static final boolean DISABLED = true;

    private TuningOpModeManager() {}

    private static OpModeMeta metaForClassAuto(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                .build();
    }

	private static OpModeMeta metaForClassTele(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

		List<Class> opModesAuto = Arrays.asList(
			ForwardVelocityTuner.class,
			ForwardZeroPowerAccelerationTuner.class,
			LateralZeroPowerAccelerationTuner.class,
			StrafeVelocityTuner.class,
			ForwardTuner.class,
			LateralTuner.class,
			TurnTuner.class,
			CurvedBackAndForth.class,
			StraightBackAndForth.class
		);

		List<Class> opModesTele = Arrays.asList(
			LocalizationTest.class,
			MotorDirections.class
		);

		for (Class<? extends OpMode> opMode : opModesAuto) {
			manager.register(metaForClassAuto(opMode), opMode);
		}

		for (Class<? extends OpMode> opMode : opModesTele) {
			manager.register(metaForClassTele(opMode), opMode);
		}

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : opModesAuto) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
            for (Class<?> c : opModesTele) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
