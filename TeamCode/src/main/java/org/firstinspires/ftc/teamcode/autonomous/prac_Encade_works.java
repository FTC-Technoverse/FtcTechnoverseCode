package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Encodercode working ")
public class prac_Encade_works extends OpMode {
    DcMotor lf_drive;
    double ticks = 537.7;
    double newTarget;
    @Override
    public void init() {
        lf_drive = hardwareMap.get(DcMotor.class,"lf_drive");
        telemetry.addData("Hardware","Initialized");
        lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        if(gamepad1.a){
            encoder(1.5);

        }
    }
    public  void  encoder(double turnage){
        lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget =ticks*turnage;
        lf_drive.setTargetPosition((int)newTarget);
        lf_drive.setPower(0.2);
        lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}