package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private boolean on = false;
    private float power = 0;

    public void enable(boolean enable) {
        if (!on && enable){
            intake.setPower(power);
        } else if (on && !enable) {
            intake.setPower(0);
        }
    }

    public void setPower(float pow) {
        power = pow;
    }

}