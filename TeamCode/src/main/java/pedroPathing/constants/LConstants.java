package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
	static {
		ThreeWheelConstants.forwardTicksToInches = 0.0029750308;
		ThreeWheelConstants.strafeTicksToInches = 0.0029520205;
		ThreeWheelConstants.turnTicksToInches = 0.0029062348;
		ThreeWheelConstants.leftY = 8;
		ThreeWheelConstants.rightY = -7.625;
		ThreeWheelConstants.strafeX = -6.5;
		ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
		ThreeWheelConstants.rightEncoder_HardwareMapName = "intakeMotor";
		ThreeWheelConstants.strafeEncoder_HardwareMapName = "leftBack";
		ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
		ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
		ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
	}
}