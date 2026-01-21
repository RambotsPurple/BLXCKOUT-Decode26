package org.firstinspires.ftc.teamcode.config.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.util.LUT;
import com.arcrobotics.ftclib.util.InterpLUT;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {

    public DcMotorEx shooter1, shooter2;
    //public InterpLUT lutShooter;
   public double f = 0 ;
    public  double p = 0;

    public ShooterSubsystem(HardwareMap hw, Telemetry t) {

        shooter1 = hw.get(DcMotorEx.class, "s1");
        shooter2 = hw.get(DcMotorEx.class, "s2");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //issue with shooter2 when encoder cable not connected
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Orientation for shooter
        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,f);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    } // init

    public void setPower(double p) {
        shooter1.setPower(p);
        shooter2.setPower(p);
    } // shoot

    //Init the Look up table

//Adding each val with a key

    // set RPM of motorv
    public void setRPM(double RPM) {
        shooter1.setVelocity(RPM * 6, AngleUnit.DEGREES);
        shooter2.setVelocity(RPM * 6, AngleUnit.DEGREES);
    }

    public double getRPM() {
        return shooter1.getVelocity(AngleUnit.DEGREES) / 6;
    }

    public double getTPS() {
        return shooter1.getVelocity();
    }

}// end of shooter class