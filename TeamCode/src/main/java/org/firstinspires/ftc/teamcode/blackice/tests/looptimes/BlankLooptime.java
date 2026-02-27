package org.firstinspires.ftc.teamcode.blackice.tests.looptimes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


// ~2ms
@Autonomous
public class BlankLooptime extends OpMode {
    private double lastTime;
    
    @Override
    public void init() {
        lastTime = System.nanoTime() * 1e-9;
    }
    
    @Override
    public void loop() {
        double now = System.nanoTime() * 1e-9;
        
        telemetry.addData("deltaTime", now - lastTime);
        telemetry.update();
        
        lastTime = now;
    }
}
