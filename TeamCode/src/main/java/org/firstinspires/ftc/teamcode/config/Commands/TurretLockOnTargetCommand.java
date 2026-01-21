package org.firstinspires.ftc.teamcode.config.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Subsystem.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.config.Subsystem.TurretSubsystem;

public class TurretLockOnTargetCommand extends CommandBase {

    private final LimeLightSubsystem limelight;
    private final TurretSubsystem turret;
    private final PIDController turnController;

    // --- TUNING VALUES ---
    private static final double Kp = 0.04; // Proportional gain - this is the main tuning value
    private static final double Ki = 0.0;  // Integral - leave 0 unless you have consistent steady-state error
    private static final double Kd = 0.003; // Derivative - helps reduce oscillation
    private static final double TOLERANCE_DEGREES = 1.0; // How close to center to be considered "finished"


    public TurretLockOnTargetCommand(LimeLightSubsystem limelight, TurretSubsystem turret) {
        this.limelight = limelight;
        this.turret = turret;
        this.turnController = new PIDController(Kp, Ki, Kd);
        this.addRequirements(turret); // do NOT add limelight - multiple commands can get data simultaneously
    }

    @Override
    public void initialize() {
        turnController.reset();
    }

    @Override
    public void execute() {
        // Get the horizontal error from the Limelight (our sensor)
        double error = limelight.getHorizontalError();

        // The controller calculates the power needed to drive the error to zero.
        double power = turnController.calculate(error, 0);

        // TODO reverse motor/power if needed
        turret.setPower(power);
    }

    // TODO rn the code likely ends if target lost, test/improve later
    @Override
    public boolean isFinished() {
        // The command is finished if we lose the target or if we are aimed correctly.
        return Math.abs(limelight.getHorizontalError()) < TOLERANCE_DEGREES;
    }

}