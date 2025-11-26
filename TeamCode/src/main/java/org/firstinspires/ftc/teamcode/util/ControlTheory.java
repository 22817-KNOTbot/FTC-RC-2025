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

		public Pid(double Kp, double Ki, double Kd) {
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			timer.reset();
		}

		public double calculate(double reference, double current) {
			return calculate(reference, current, null);
		}

		public double calculate(double reference, double current, TelemetryManager telemetryManager) {
			double error = reference - current;
			double difference;
			if (lastError != null) {
				difference = error - lastError;
			} else {
				difference = 0;
			}
			double derivative = difference / timer.seconds();
			integralSum = integralSum + (error * timer.seconds());

			double output = (Kp * error) +
					(Ki * integralSum) +
					(Kd * derivative);

			lastError = error;
			timer.reset();

			if (telemetryManager != null) {
				telemetryManager.addData("Error", error);
				telemetryManager.addData("ref", reference);
				telemetryManager.addData("cur", current);
				telemetryManager.addData("lastError", lastError);
				telemetryManager.addData("calculated difference", difference);
			}

			return output;
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
}
