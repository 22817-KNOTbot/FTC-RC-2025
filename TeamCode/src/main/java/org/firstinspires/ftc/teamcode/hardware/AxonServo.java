package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import android.util.Log;

public class AxonServo implements Servo {
	public double halfTurnDelayMs = 200;

	private ServoImplEx servo;

	private double targetPosition = 0;
	private double lastPosition = 0;
	private long lastSetTime = 0;

	public AxonServo(Servo servo) {
		this.servo = (ServoImplEx) servo;
		this.servo.setPwmRange(new ServoImplEx.PwmRange(500, 2500));
	}

	@Override
	public void setPosition(double position) {
		if (targetPosition == position) return;
		targetPosition = position;
		double target = 0;
		if (Math.abs(position - lastPosition) >= 0.5) {
			target = Math.signum(position - lastPosition) * 0.49 + lastPosition;
			Log.d("AxonServo", String.format("Used 2 positions for %f from %f; Used %f", position, lastPosition, target));
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
