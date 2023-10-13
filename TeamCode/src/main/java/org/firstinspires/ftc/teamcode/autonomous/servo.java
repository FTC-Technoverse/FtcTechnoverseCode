package org.firstinspires.ftc.teamcode.autonomous;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servo")
public class servo extends OpMode {
    //    public CRServo servo;
    public Servo servo2;

    int x=0;
    DcMotor lf_drive;
    double ticks = 537.7;
    double newTarget;

    @Override
    public void init() {
        servo2 = hardwareMap.get(Servo.class, "servo");
        lf_drive = hardwareMap.get(DcMotor.class,"lf_drive");
        telemetry.addData("Hardware","Initialized");
        lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {

        if(gamepad1.a){
            servo2.setPosition(0);

        }
        if(gamepad1.b){
            while(x<3){

                servo2.setPosition(0.33);
                sleep(1000);
//                encoder(1.5);
                servo2.setPosition(0.66);
                sleep(1000);
//                encoder(1.5);
                servo2.setPosition(0.99);
                sleep(1000);
//                encoder(1.5);
                servo2.setPosition(0);
                sleep(2000);
//                encoder(1.5);
                x+=1;


            }
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