package org.firstinspires.ftc.teamcode.config.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.Util.Alliance;

import java.util.List;

public class LimeLightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    private Alliance alliance;
    private final int id;
    public  LimeLightSubsystem(HardwareMap hw, Alliance alliance) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        this.alliance = alliance;
        id = this.alliance == Alliance.RED ? 24 : 20;
    } //end of constructor

    public void lStart(){
        limelight.start();
    }//end of lStart

    // TODO optimize by combining 2 very similar get into one method
    public double getDist() {
        FiducialResult tag = null;
        double tagDist = 0;
        List<FiducialResult> results = limelight.getLatestResult().getFiducialResults();

        if (results.isEmpty()) return 0;

        for (FiducialResult fiducial : results) {
            if (fiducial!=null &&fiducial.getFiducialId() == id) {
                tag = fiducial;
                break;
            }//end of if
        }//end of for

        if(tag !=null){
            tagDist = (tag.getCameraPoseTargetSpace().getPosition().z/ DistanceUnit.mPerInch);
        }//end of if
        return tagDist;

    }//end getDist

    public double getHorizontalError(){
        FiducialResult tag = null;
        double tagYaw = 0;
        List<FiducialResult> results = limelight.getLatestResult().getFiducialResults();

        if (results.isEmpty()) return 0;

        for (FiducialResult fiducial : results) {
            if (fiducial != null && fiducial.getFiducialId() == id){
                tag = fiducial;
                break;
            }//end of if
        }//end of for

        if(tag !=null){
            tagYaw = tag.getTargetXDegrees();
        }//end of if

        return tagYaw;
    } //end of getHoriError


}//end of LimeLightSubsystem
