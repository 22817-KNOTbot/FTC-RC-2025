package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
	static {
		ThreeWheelConstants.forwardTicksToInches = .001989436789;
		ThreeWheelConstants.strafeTicksToInches = .001989436789;
		ThreeWheelConstants.turnTicksToInches = .001989436789;
		ThreeWheelConstants.leftY = 8.3;
		ThreeWheelConstants.rightY = -8.3;
		ThreeWheelConstants.strafeX = -8.3;
		ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
		ThreeWheelConstants.rightEncoder_HardwareMapName = "intakeMotor";
		ThreeWheelConstants.strafeEncoder_HardwareMapName = "leftBack";
		ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
		ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
		ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
	}
}