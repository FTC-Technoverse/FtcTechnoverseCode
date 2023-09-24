package org.firstinspires.ftc.teamcode.teleop;

public class Point {
    private double r, theta; 
     public Point(double r, double theta) {
        this.r = r;
        this.theta = theta;
    }
    public double getR() {
         return r;
    } 
    public double getTheta() {
         return theta;
    }
}
