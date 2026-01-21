package org.firstinspires.ftc.teamcode.config.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem extends SubsystemBase {

    public CRServo turd;
    //public  InterpLUT lutHood;
    public TurretSubsystem(HardwareMap hw, Telemetry t) {
        //need to go on the field a tune key points top of the key inside paint and from half
        //lutHood = new InterpLUT();
        // lutHood.add(1.1, 0.2);
        // lutHood.add(2.7, .5);
        // lutHood.add(3.6, 0.75);
        // lutHood.add(4.1, 0.9);
        // lutHood.add(5, 1);
        //lutHood.createLUT(); //calc the cubuic
        turd = hw.get(CRServo.class, "hood");
    } // init

    public void setPower(double p) {
        turd.setPower(p);
    } // setPositon



}// end of HoodSubsystem