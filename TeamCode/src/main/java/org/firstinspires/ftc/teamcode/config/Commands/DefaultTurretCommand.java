package org.firstinspires.ftc.teamcode.config.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Subsystem.TurretSubsystem;

public class DefaultTurretCommand extends CommandBase {

    private final GamepadEx op;
    private final TurretSubsystem turret;


    public DefaultTurretCommand(GamepadEx op, TurretSubsystem turret) {
        this.op = op;
        this.turret = turret;
        this.addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        turret.setPower(Math.pow(op.getRightX(), 3));
    }

}