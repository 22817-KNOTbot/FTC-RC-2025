package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class ControlTheory {
    public static class PID {
        private double Kp;
        private double Ki;
        private double Kd;
        private boolean debug;
        private FtcDashboard dashboard;
        public TelemetryPacket telemetryPacket;    

        private ElapsedTime timer = new ElapsedTime();
        private double integralSum = 0;
        private double lastError = 0;

        public PID(double Kp, double Ki, double Kd) {
            this(Kp, Ki, Kd, false);
        }

        public PID(double Kp, double Ki, double Kd, boolean debug) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.debug = debug;
            this.dashboard = FtcDashboard.getInstance(); 
            this.telemetryPacket = new TelemetryPacket();
            timer.reset();
        }

        public double calculate(double reference, double current) {
            double error = reference - current;
            double derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());

            double output = 
                (Kp * error) + 
                (Ki * integralSum) + 
                (Kd * derivative);
            
            lastError = error;
            timer.reset();

            if (debug) {
                telemetryPacket.put("Error", error);
                telemetryPacket.put("ref", reference);
                telemetryPacket.put("cur", current);
                dashboard.sendTelemetryPacket(telemetryPacket);
                telemetryPacket = new TelemetryPacket();
            }
            
            return output;
        }

        public void resetIntegral() {
            integralSum = 0;
        }
    }
}
