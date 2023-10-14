package org.firstinspires.ftc.teamcode.autonomous;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Encodercode working ")
public class prac_Encade_works extends OpMode {
    DcMotor lf_drive;
    DcMotor rf_drive;
//    DcMotor lb_drive;
//    DcMotor rb_drive;

    double ticks = 537.7;
    double newTargetlf;
    double newTargetrf;
    @Override
    public void init() {
        lf_drive = hardwareMap.get(DcMotor.class,"lf_drive");
        rf_drive = hardwareMap.get(DcMotor.class,"rf_drive");
//        lb_drive = hardwareMap.get(DcMotor.class,"lb_drive");
//        rb_drive = hardwareMap.get(DcMotor.class,"rb_drive");

        telemetry.addData("Hardware","Initialized");
        lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lf_drive.setDirection(DcMotor.Direction.REVERSE);
//        lb_drive.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            encoder(1,1);
            
        }

    }





    public  void  encoder(double turnage1,double turnage2){
        lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newTargetlf =ticks*turnage1;
        newTargetrf =ticks*turnage2;

        lf_drive.setTargetPosition((int)newTargetlf);
        rf_drive.setTargetPosition((int)newTargetrf);
//        lb_drive.setTargetPosition((int)newTarget);
//        rb_drive.setTargetPosition((int)newTarget);

        lf_drive.setPower(0.1);
        rf_drive.setPower(0.1);
//        lb_drive.setPower(0.2);
//        rb_drive.setPower(0.2);

        lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
}