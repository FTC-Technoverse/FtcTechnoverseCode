package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
public class Drivetrain {
    private DcMotor lfdrive;

    private DcMotor rfdrive;

    private DcMotor lbdrive;

    private DcMotor rbdrive;

    private double leftPower;
    private double rightPower;



    public Drivetrain() {

        lfdrive  = hardwareMap.get(DcMotor.class, "lf_drive");
        rfdrive = hardwareMap.get(DcMotor.class, "rf_drive");
        lbdrive  = hardwareMap.get(DcMotor.class, "lb_drive");
        rbdrive = hardwareMap.get(DcMotor.class, "rb_drive");

        lfdrive.setDirection(DcMotor.Direction.REVERSE);
        rfdrive.setDirection(DcMotor.Direction.FORWARD);
        lbdrive.setDirection(DcMotor.Direction.REVERSE);
        rbdrive.setDirection(DcMotor.Direction.REVERSE);

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        lfdrive.setPower(leftPower);
        rfdrive.setPower(rightPower);
        lbdrive.setPower(leftPower);
        rbdrive.setPower(rightPower);
    }

    public DcMotor getLfDrive() {
        return lfdrive;
    }
    public DcMotor getRfDrive() {
        return rfdrive;
    }
    public DcMotor getLbDrive() {
        return lbdrive;
    }
    public DcMotor getRbDrive() {
        return rbdrive;
    }
    public double getLeftPower() {
        return leftPower;
    }
    public double getRightPower() {
        return rightPower;
    }


}
