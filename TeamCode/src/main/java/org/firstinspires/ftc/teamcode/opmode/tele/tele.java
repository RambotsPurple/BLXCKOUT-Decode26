package org.firstinspires.ftc.teamcode.opmode.tele;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Util.Alliance;
@TeleOp(name = "tele")
public class tele extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1, gamepad2, telemetry);
        robot.tele();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()){
            robot.alliance = Alliance.BLUE;
        } else if (gamepad1.bWasPressed()) {
            robot.alliance = Alliance.RED;
        }
        telemetry.addLine("press X to change to BLUE Alliance");
        telemetry.addLine("press B to change to RED Alliance");
        telemetry.addData("current alliance selected:", robot.alliance);
        telemetry.update();
    }

    @Override
    public void start(){
        robot.tStart();
    }

    @Override
    public void loop() {
        robot.tPeriodic();
    }
}
