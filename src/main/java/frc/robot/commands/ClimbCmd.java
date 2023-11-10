package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.AutonomousConstants.headingPIDConstants;
import frc.robot.Constants.AutonomousConstants.pitchPidConstants;

//TODO: Test this command on the robot to make sure it works
public class ClimbCmd extends CommandBase{
    private DriveSubsystem driveSubsystem;
    private PIDController headingPidController;
    private PIDController pitchPidController;
    private Timer timer;
    private double driveSpeed;
    private double timeout;
    private boolean reversed;
    private boolean finished;
    private boolean atClimbingAngle;
    private boolean stabiliseForever;
    
    /**
     * Makes the Robot Climb to the balance board.
     * Does so by using the gyro to keep the robot straight and the pitch to keep the robot at the norml angle.
     * Make sure to use this while we are facing foward to the balance board.
     * Recommended to use this command after the robot has been driven to the balance board or at least close to it with a low timeout.
     * WARNING: This command will continue going foward until şt reaches the balance board, So do not run it if not in front of the balance board.
     * @param driveSubsystem The drive subsystem to be used by this command.
     * @param timeout The timeout in seconds. this will stop the command if the command takes too long to finish. Meaining we probably missed the board.
     * @param isReverse If true, the robot will go backwards. If false, the robot will go foward.
     */
    public ClimbCmd(DriveSubsystem driveSubsystem, double timeout, boolean isReverse, boolean stabiliseForever) {
        this.driveSubsystem = driveSubsystem;
        this.reversed = isReverse;
        this.timeout = timeout;
        this.stabiliseForever = stabiliseForever;
        timer = new Timer();
        timer.reset();
        if (isReverse) {
            this.driveSpeed = -AutonomousConstants.kDriveSpeed;
        }
        headingPidController = new PIDController(headingPIDConstants.kP, headingPIDConstants.kI, headingPIDConstants.kD);
        pitchPidController = new PIDController(pitchPidConstants.kP, pitchPidConstants.kI, pitchPidConstants.kD);
        addRequirements(driveSubsystem);
    }

    /**
     * Makes the Robot Climb to the balance board.
     * Does so by using the gyro to keep the robot straight and the pitch to keep the robot at the norml angle.
     * Make sure to use this while we are facing foward to the balance board.
     * Recommended to use this command after the robot has been driven to the balance board or at least close to it with a low timeout.
     * WARNING: This command will continue going foward until şt reaches the balance board, So do not run it if not in front of the balance board.
     * @param driveSubsystem The drive subsystem to be used by this command.
     * @param timeout The timeout in seconds. this will stop the command if the command takes too long to finish. Meaining we probably missed the board.
    */
    public ClimbCmd(DriveSubsystem driveSubsystem, double timeout) {
        this(driveSubsystem, timeout, false, true);
    }

    @Override
    public void initialize() {
        System.out.println("Stabilise Auto Started!");
        timer.reset();
        timer.start();
        // Setup booleans
        atClimbingAngle = false;
        finished = false;
        // Setup the PID Controllers
        headingPidController.setSetpoint(driveSubsystem.getAngle());
        headingPidController.setTolerance(headingPIDConstants.kToleranceDegrees);
        
        pitchPidController.setTolerance(pitchPidConstants.kToleranceDegrees);
        pitchPidController.setSetpoint(driveSubsystem.getPitch());
    }

    @Override
    public void execute() {
        // Go Foward till the robot looks up
        // We might need to add a timeout to this
        if (pitchPidController.atSetpoint() && !atClimbingAngle) {
            if (timer.get() > this.timeout) {
                // We took too long to climb
                // We probably missed the board
                // Stop the command And Return an error
                System.out.println("Stabilise Auto Timed Out!");
                DriverStation.reportError("Stabilise Auto Timed Out! Stopping", false);
                driveSubsystem.stopMotors();
                finished = true;
                return;
            }
            driveSubsystem.drive(driveSpeed, 0, false);
        }
        else {
            double pitchCorrection = reversed ? -pitchPidController.calculate(driveSubsystem.getPitch()) : pitchPidController.calculate(driveSubsystem.getPitch());
            // We started Climbing
            // Try to keep the robot straight
            atClimbingAngle = true;
            driveSubsystem.drive(pitchCorrection, headingPidController.calculate(driveSubsystem.getAngle()), false);
            if (!stabiliseForever) finished = pitchPidController.atSetpoint(); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        System.out.println("Stabilise Auto Ended!" + (interrupted ? " And was Interrupted!" : ""));
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
