package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Config
@TeleOp(name="Hub LED Control", group="Debug")
public class HubLedTesting extends LinearOpMode {
    public static int red = 255;
    public static int green = 255;
    public static int blue = 255;
    private List<LynxModule> hubs;

    @Override
    public void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);

        waitForStart();

        while (opModeIsActive()) {
            int colour = (red << 16) | (green << 8) | blue;
            for (LynxModule hub : hubs) {
                hub.setConstant(colour);
            }    
        }
    }
}