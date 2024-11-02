package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
    var requirementsList = List.of(robot.intake, robot.shooter, robot.climber);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command idleCommand() {
    return Commands.runOnce(robot::idleRequest, requirements).withName("IdleCommand");
  }

  public Command intakeCommand() {
    return Commands.runOnce(robot::intakeRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_W_GP))
        .withName("IntakeCommand");
  }

  public Command outtakeCommand() {
    return Commands.runOnce(robot::outtakeRequest, requirements).withName("OuttakeCommand");
  }

  public Command outtakeShooterCommand() {
    return Commands.runOnce(robot::outtakeShooterRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP))
        .withName("OuttakeShooterCommand");
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, requirements).withName("UnjamCommand");
  }

  public Command waitSubwooferCommand() {
    return Commands.runOnce(robot::waitSubwooferShotRequest, requirements)
        .withName("WaitingSubwooferCommand");
  }

  public Command waitSpeakerCommand() {
    return Commands.runOnce(robot::waitSpeakerShotRequest, requirements)
        .withName("WaitingSpeakerCommand");
  }

  public Command waitFloorCommand() {
    return Commands.runOnce(robot::waitFloorShotRequest, requirements)
        .withName("WaitingFloorCommand");
  }

  public Command confirmShotCommand() {
    return Commands.runOnce(robot::confirmShotRequest, requirements).withName("ConfirmShotCommand");
  }

  public Command waitAmpCommand() {
    return Commands.runOnce(robot::waitAmpRequest, requirements).withName("AmpCommand");
  }

  public Command speakerShotCommand() {
    return Commands.runOnce(robot::speakerShotRequest, requirements).withName("SpeakerShotCommand");
  }

  public Command nextClimbCommand() {
    return Commands.runOnce(robot::nextClimbRequest, requirements).withName("ClimbSequenceCommand");
  }

  public Command reverseClimbCommand() {
    return Commands.runOnce(robot::reverseClimbRequest, requirements)
        .withName("ClimbSequenceCommand");
  }

  public Command homeClimberCommand() {
    return Commands.runOnce(robot::homeClimberRequest, requirements).withName("HomeClimberCommand");
  }

  public Command preloadNoteCommand() {
    return Commands.runOnce(robot::preloadNoteRequest, requirements).withName("preloadNoteCommand");
  }
}
