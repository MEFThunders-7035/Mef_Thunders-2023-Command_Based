package frc.robot.abstract_classes;

public class PIDConstants {
    public double kP;
    public double kI;
    public double kD;
    
    public PIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDConstants() {
        this(0, 0, 0);
    }

    public PIDConstants(PIDConstants pidConstants) {
        this(pidConstants.kP, pidConstants.kI, pidConstants.kD);
    }
}
