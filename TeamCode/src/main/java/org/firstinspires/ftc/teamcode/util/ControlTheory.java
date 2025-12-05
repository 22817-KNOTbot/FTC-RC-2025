package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ControlTheory {
	public static class Pid {
		private double Kp;
		private double Ki;
		private double Kd;

		private ElapsedTime timer = new ElapsedTime();
		private double integralSum = 0;
		private Double lastError = null;

		private double reference;
		private double current;
		private double error;
		private double difference;
		private double derivative;
		private double output;

		public Pid(double Kp, double Ki, double Kd) {
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			timer.reset();
		}

		public double calculate(double reference, double current) {
			error = reference - current;
			if (lastError != null) {
				difference = error - lastError;
			} else {
				difference = 0;
			}
			derivative = difference / timer.seconds();
			integralSum = integralSum + (error * timer.seconds());

			output = (Kp * error) +
					(Ki * integralSum) +
					(Kd * derivative);

			lastError = error;
			timer.reset();

			return output;
		}

		public void showTelemetry(TelemetryManager telemetry) {
				telemetry.addData("Error", error);
				telemetry.addData("ref", reference);
				telemetry.addData("cur", current);
				telemetry.addData("lastError", lastError);
				telemetry.addData("calculated difference", difference);
		}

		public void resetIntegral() {
			integralSum = 0;
		}

		public void resetLastError() {
			lastError = null;
		}

		public void setKp(double Kp) {
			this.Kp = Kp;
		}

		public void setKi(double Ki) {
			this.Ki = Ki;
		}

		public void setKd(double Kd) {
			this.Kd = Kd;
		}
	}

	public static class Pidf {
		private double Kp;
		private double Ki;
		private double Kd;
		private double Kv;

		private ElapsedTime timer = new ElapsedTime();
		private double integralSum = 0;
		private Double lastError = null;

		private double reference;
		private double current;
		private double error;
		private double difference;
		private double derivative;
		private double output;

		public Pidf(double Kp, double Ki, double Kd, double Kv) {
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			this.Kv = Kv;
			timer.reset();
		}

		public double calculate(double reference, double current) {
			error = reference - current;
			if (lastError != null) {
				difference = error - lastError;
			} else {
				difference = 0;
			}
			derivative = difference / timer.seconds();
			integralSum = integralSum + (error * timer.seconds());

			output = (Kp * error) +
					(Ki * integralSum) +
					(Kd * derivative) +
					(Kv * reference);

			lastError = error;
			timer.reset();

			return output;
		}

		public void showTelemetry(TelemetryManager telemetry) {
				telemetry.addData("Error", error);
				telemetry.addData("ref", reference);
				telemetry.addData("cur", current);
				telemetry.addData("lastError", lastError);
				telemetry.addData("calculated difference", difference);
		}

		public void resetIntegral() {
			integralSum = 0;
		}

		public void resetLastError() {
			lastError = null;
		}

		public void setKp(double Kp) {
			this.Kp = Kp;
		}

		public void setKi(double Ki) {
			this.Ki = Ki;
		}

		public void setKd(double Kd) {
			this.Kd = Kd;
		}

		public void setKv(double Kv) {
			this.Kv = Kv;
		}
	}
}
