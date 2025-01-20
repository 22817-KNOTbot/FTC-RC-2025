package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@Config
// @TeleOp(name="Hub LED Control", group="Debug")
public class HubLedTesting extends LinearOpMode {
    public static int red = 255;
    public static int green = 255;
    public static int blue = 255;
    public static int MODE = 0;
    private List<LynxModule> hubs;

    @Override
    public void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);

        waitForStart();

        while (opModeIsActive()) {
            switch (MODE) {
                case 0:
                    for (LynxModule hub : hubs) {
                        hub.stopBlinking();
                    }  
                    break;  
                case 1:
                    for (LynxModule hub : hubs) {
                        hub.setConstant(convertRGB(red, green, blue));
                    }    
                    break;
                case 2:
                    Collection<Blinker.Step> steps = new ArrayList<>();
                    // K
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 300, TimeUnit.MILLISECONDS));
                    
                    // N
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 100, TimeUnit.MILLISECONDS));
                    
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 300, TimeUnit.MILLISECONDS));
                    
                    // O
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 300, TimeUnit.MILLISECONDS));
                    
                    // T
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));

                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 300, TimeUnit.MILLISECONDS));

                    // B
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));

                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 300, TimeUnit.MILLISECONDS));

                    // O
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 100, TimeUnit.MILLISECONDS));
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));
                    
                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 300, TimeUnit.MILLISECONDS));

                    // T
                    steps.add(new Blinker.Step(convertRGB(255, 255, 255), 300, TimeUnit.MILLISECONDS));

                    steps.add(new Blinker.Step(convertRGB(0, 0, 0), 300, TimeUnit.MILLISECONDS));

                    for (LynxModule hub : hubs) {
                        hub.setPattern(steps);
                    }    
                    break;
            }
        }
    }

    private int convertRGB(int red, int green, int blue) {
        return (red << 16) | (green << 8) | blue;
    }
}