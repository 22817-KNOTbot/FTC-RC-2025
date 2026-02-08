package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class AxonServo implements Servo {
	public static final double FULL_TURN_VOLTAGE = 3.3;
	public double halfTurnDelayMs = 400;
	public double fullTurnDegrees = 360;

	private ServoImplEx servo;

	private double targetPosition = 0;
	private double lastPosition = 0;
	private long lastSetTime = 0;

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
		if (targetPosition == position) return;
		targetPosition = position;
		double target = 0;
		if (Math.abs(position - lastPosition) >= 0.5) {
			target = Math.signum(position - lastPosition) * 0.49 + lastPosition;
		} else {
			target = position;
		}
		servo.setPosition(target);
		lastPosition = target;
		lastSetTime = System.currentTimeMillis();
	}

	public void update() {
		if (targetPosition != lastPosition && System.currentTimeMillis() - lastSetTime > halfTurnDelayMs) {
			servo.setPosition(targetPosition);
			lastPosition = targetPosition;
		}
	}

	/**
	 * Returns null if analog device not given
	 */
	public Double getAngle() {
		if (analogInput != null && analogInput.getVoltage() != 0) {
			return (analogInput.getVoltage() / FULL_TURN_VOLTAGE) * fullTurnDegrees;
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
