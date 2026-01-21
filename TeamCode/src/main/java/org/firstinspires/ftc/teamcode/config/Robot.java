package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.Commands.DefaultTurretCommand;
import org.firstinspires.ftc.teamcode.config.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.config.Commands.SetShooterVelocityCommand;
import org.firstinspires.ftc.teamcode.config.Util.*;
import org.firstinspires.ftc.teamcode.config.Subsystem.*;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;

import java.util.List;

public class Robot {

    private final List<LynxModule> allHubs;
    private final Timer loop = new Timer();
    public Alliance alliance;
    public CommandScheduler cs = CommandScheduler.getInstance();

    protected GamepadEx driver;
    protected GamepadEx operator;

    // Instantiate subsystems
    public ShooterSubsystem shooterSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public TransferSubsystem transferSubsystem;
    public DriveSubsystem driveSubsystem;
    public TurretSubsystem turretSubsystem;

    public LimeLightSubsystem limeLightSubsystem;
    public IndexerSubsystem indexerSubsystem;
    private final Telemetry telemetry;
    public state state;

    //for pd control for auto alignment
    public double error = 0 ;
    public double lastError = 0 ;

    public double kp = 0;
    public double kd =0;

    public Follower getFollower(){
        return follower;
    } // end of getFollower


    /**
     * This constructor below is for a single player tele anywhere
     * of the field
     ** @param h hardwaremap
     * @param alliance blue or red
     * @param driver driver gamepad
     * @param telemetry allows for telemetry output
     **/
    public Robot(HardwareMap h, Alliance alliance, Gamepad driver, Gamepad operator, Telemetry telemetry) {
        shooterSubsystem = new ShooterSubsystem(h,telemetry);
        intakeSubsystem = new IntakeSubsystem(h);
        transferSubsystem = new TransferSubsystem(h);
        limeLightSubsystem = new LimeLightSubsystem(h,alliance);
        turretSubsystem = new TurretSubsystem(h,telemetry);
        indexerSubsystem = new IndexerSubsystem(h,telemetry);
        follower = Constants.createFollower(h);
        follower.setStartingPose(new Pose(0,0,0));
        this.alliance = alliance;
        this.driver = new GamepadEx(driver);
        this.operator = new GamepadEx(operator);
        this.telemetry = telemetry;

        allHubs = h.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }//end of for

        loop.resetTimer();
        cs.registerSubsystem(
                shooterSubsystem, transferSubsystem, intakeSubsystem,limeLightSubsystem, turretSubsystem, indexerSubsystem
        );

        // set default commands
        turretSubsystem.setDefaultCommand(new DefaultTurretCommand(this.operator, turretSubsystem));


    } //end of teleop constructor

    /**
     * The constructor below is for auto on any side of the field
     *
     * @author Alex
     * @param h
     * @param alliance
     * @param telemetry
     */

    public Robot(HardwareMap h, Alliance alliance, Telemetry telemetry) {
        shooterSubsystem = new ShooterSubsystem(h, telemetry);
        intakeSubsystem = new IntakeSubsystem(h);
        transferSubsystem = new TransferSubsystem(h);
        limeLightSubsystem = new LimeLightSubsystem(h, alliance);
        turretSubsystem = new TurretSubsystem(h, telemetry);
        indexerSubsystem = new IndexerSubsystem(h, telemetry);

        follower = Constants.createFollower(h);
        follower.setStartingPose(new Pose(0,0,0));
        // pc(follower);
        this.alliance = alliance;
        this.telemetry = telemetry;

        // instaniate the lynx mod
        allHubs = h.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }//end of for

        cs.registerSubsystem(
                shooterSubsystem, transferSubsystem, intakeSubsystem, limeLightSubsystem, turretSubsystem, indexerSubsystem
        ); // end of cs

    }//end of teleop constructor

    /**
     * loops periodically during teleOp
     */
    public void tPeriodic() {
        teleTelemetry();

        //every 5 milliseconds clear the cache
        if (loop.getElapsedTime() % 5 == 0) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            } // end of for
        } // end of if

        // logic for auto tracking
        double turn;

        if (state != state.none){
            ElapsedTime timer = new ElapsedTime();
            error = limeLightSubsystem.getHorizontalError();
            turn = trackTo(error, timer);
        } else {
            turn = -driver.getRightX();
        } // end of if..else

        // params for drive
        // TODO tune and use ts later bru
        follower.setTeleOpDrive(
                -driver.getLeftX() ,
                -driver.getLeftY() ,
                turn,
                false
        );

        // TODO test command and then remove ts completely
        // manual turret control - conflicts with commands
//        turretSubsystem.setPower(Math.pow(operator.getRightX(), 3));

        follower.update();
        telemetry.update();
        cs.run();
    } //end of periodic

    /**
     * Run on start of teleOp
     */
    public void tStart() {
        state = state.idle;
        limeLightSubsystem.lStart();
        follower.update();
        follower.startTeleopDrive(true);
    } // end of tStart

    /**
     * We use the error from our limelight for our pid and use that to adjust
     * to the april tag accordingly
     * @param error
     * @param time
     * @return pow
     */
    public double trackTo(double error, ElapsedTime time) {
        double d = (error - lastError) / time.seconds();
        double pow = (kp*error)+(kd*d);

        lastError = error;
        time.reset();
        return pow;
    } // end of trackTo

    /**
     * Sets up listeners at the start of teleOp
     */
    public void tele() {

        // event listeners
        // all input goes here except for driving which is passed in periodically
        // also turret in periodic
        // --------------------

        /*
         * DRIVER:
         * right trigger - hold: intake
         * A - activate shooter
         * B - deactivate shooter
         */

        /*
         * OPERATOR:
         * right stick = move turret
         */

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0)
                .whenActive(new IntakeCommand(intakeSubsystem, 1))
                .whenInactive(new IntakeCommand(intakeSubsystem, 0));

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SetShooterVelocityCommand(shooterSubsystem, 6000)
        );

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SetShooterVelocityCommand(shooterSubsystem, 0)
        );

        // what is ts
        // driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        //         .toggleWhenActive(state = state.idle, state = state.none);

    } //end of tele method

    /**
     * Telemetry data for teleOp
     */
    public void teleTelemetry() {
        // telemetry.addData("ticks", shooterSubsystem.getCurrentPosition());
        telemetry.addData("input", shooterSubsystem.shooter1.getPower());
        telemetry.addData("heading", driveSubsystem.getAngle());
        telemetry.addData("shooter 1 vel", shooterSubsystem.shooter1.getVelocity());
        telemetry.addData("shooter 2 vel", shooterSubsystem.shooter2.getVelocity());
        telemetry.addData("shooter RPM", shooterSubsystem.getRPM());
        telemetry.addData("servo position", indexerSubsystem.indexer.getPosition());
    } //end of teleTelemetry

    /**
     * Telemetry data for autoOp
     */
    public void autoTelemetry() {

    } //end of autoTelemetry

    /**
     * loops periodically during autoOp
     */
    public void aPeriodic(){
        autoTelemetry();
        if (loop.getElapsedTime() % 5 == 0) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            } // end of for
        } // end of if
        follower.update();
        cs.run();
    } //end of aPeriodic

    // TODO maybe nuke this idk man im gonna dieeeeeeeeeee
    public void badDrive() {
        driveSubsystem.drive(0, 0, 0);
    }

    /**
     * Runs on the start of autoOp
     */
    public void aStart(){
        state = state.idle;
        limeLightSubsystem.lStart();
    } //end of aStart
} //end of Robot