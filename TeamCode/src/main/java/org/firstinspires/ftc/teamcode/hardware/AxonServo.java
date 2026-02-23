package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@Configurable
public class AxonServo implements Servo {
	public static final double FULL_TURN_VOLTAGE = 3.3;
	public static double defaultHalfTurnDelayMs = 400;
	public static double defaultFullTurnDegrees = 360;
	public static double maxAnalogAngleChange = 160;
	public double halfTurnDelayMs = defaultHalfTurnDelayMs;
	public double fullTurnDegrees = defaultFullTurnDegrees;

	private ServoImplEx servo;

	private double targetPosition = 0;
	private double targetMidPosition = 0;
	private Double lastPosition = null;
	private long lastSetTime = 0;
	private int direction = 0;
	private boolean waitingPosition = false;

	private Double lastAnalogAngle = null;

	private AnalogInput analogInput;

	public AxonServo(Servo servo) {
		this(servo, null);
	}

	public AxonServo(Servo servo, AnalogInput analogInput) {
		this.servo = (ServoImplEx) servo;
		this.servo.setPwmRange(new ServoImplEx.PwmRange(500, 2500));
		this.analogInput = analogInput;
	}

	@Override
	public void setPosition(double position) {
		if (lastPosition == null) {
			lastPosition = position;
		}
		if (targetPosition == position) return;
		targetPosition = position;
		double target = 0;
		if (waitingPosition && (int) Math.signum(position - targetMidPosition) == direction) {
			return;
		} else if (Math.abs(position - lastPosition) >= 0.5) {
			target = Math.signum(position - lastPosition) * 0.49 + lastPosition;
			waitingPosition = true;
		} else {
			target = position;
		}
		targetMidPosition = target;
		servo.setPosition(target);
		lastPosition = target;
		lastSetTime = System.currentTimeMillis();
		direction = (int) Math.signum(position - lastPosition);
	}

	public void update() {
		if (waitingPosition && System.currentTimeMillis() - lastSetTime > halfTurnDelayMs) {
			servo.setPosition(targetPosition);
			lastPosition = targetPosition;
			waitingPosition = false;
		}
	}

	/**
	 * Returns null if analog device not given
	 */
	public Double getAngle() {
		if (analogInput != null && analogInput.getVoltage() != 0) {
			double angle = (analogInput.getVoltage() / FULL_TURN_VOLTAGE) * fullTurnDegrees;
			if (lastAnalogAngle == null || Math.abs(angle - lastAnalogAngle) <= maxAnalogAngleChange) {
				lastAnalogAngle = angle;
			}
			return lastAnalogAngle;
		} else {
			return null;
		}
	}
	
	/**
	 * Returns null if analog device not given
	 */
	public Double getVoltage() {
		if (analogInput != null) {
			return analogInput.getVoltage();
		} else {
			return null;
		}
	}

	public void enablePwm() {
		servo.setPwmEnable();
	}

	public void disablePwm() {
		servo.setPwmDisable();
	}

	@Override
	public ServoController getController() {
		return servo.getController();
	}

	@Override
	public Direction getDirection() {
		return servo.getDirection();
	}

	@Override
	public int getPortNumber() {
		return servo.getPortNumber();
	}

	@Override
	public double getPosition() {
		return servo.getPosition();
	}

	@Override
	public void scaleRange(double min, double max) {
		servo.scaleRange(min, max);
	}

	@Override
	public void setDirection(Direction direction) {
		servo.setDirection(direction);
	}

	@Override
	public void close() {
		servo.close();
	}

	@Override
	public String getConnectionInfo() {
		return servo.getConnectionInfo();
	}

	@Override
	public String getDeviceName() {
		return servo.getDeviceName();
	}

	@Override
	public Manufacturer getManufacturer() {
		return servo.getManufacturer();
	}

	@Override
	public int getVersion() {
		return servo.getVersion();
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
		servo.resetDeviceConfigurationForOpMode();
	}	
}
