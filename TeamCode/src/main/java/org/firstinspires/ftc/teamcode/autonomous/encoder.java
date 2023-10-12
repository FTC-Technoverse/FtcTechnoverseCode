package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Encoder Test ")
public class encoder extends LinearOpMode {
    DcMotor lfdrive;
    DcMotor rfdrive;
    DcMotor lbdrive;
    DcMotor rbdrive;
    double speed= 0.3;
    double ticks = 537.7;
    double newLfTarget;
    double newRfTarget;
    double newLbTarget;
    double newRbTarget;


    public void runOpMode() {
        lfdrive = hardwareMap.get(DcMotor.class, "lf_drive");
        rfdrive = hardwareMap.get(DcMotor.class, "rf_drive");
        lbdrive = hardwareMap.get(DcMotor.class, "lb_drive");
        rbdrive = hardwareMap.get(DcMotor.class, "rb_drive");

        lfdrive.setDirection(DcMotor.Direction.REVERSE);
        rfdrive.setDirection(DcMotor.Direction.FORWARD);
        lbdrive.setDirection(DcMotor.Direction.REVERSE);
        rbdrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Hardware: ", "Initialized");
        lfdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        turnEncoder(1);

        telemetry.addData("Motor Ticks: ", lfdrive.getCurrentPosition());


    }


    public void turnEncoder(int turnage){
        newLfTarget = ticks*turnage;
        newRfTarget = ticks*turnage;
        newLbTarget = ticks*turnage;
        newRbTarget = ticks*turnage;

        lfdrive.setTargetPosition((int)newLfTarget);
        rfdrive.setTargetPosition((int)newRfTarget);
        lbdrive.setTargetPosition((int)newLbTarget);
        rbdrive.setTargetPosition((int)newRbTarget);

        lfdrive.setPower(speed);
        rfdrive.setPower(speed);
        lbdrive.setPower(speed);
        rbdrive.setPower(speed);

        lfdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void tracker(){
        lfdrive.setTargetPosition(0);
        rfdrive.setTargetPosition(0);
        lbdrive.setTargetPosition(0);
        rbdrive.setTargetPosition(0);

        lfdrive.setPower(speed);
        rfdrive.setPower(speed);
        lbdrive.setPower(speed);
        rbdrive.setPower(speed);

        lfdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

}