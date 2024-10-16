package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.Set;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;

  private Set<RobotState> idleStates = Set.of(RobotState.IDLE_W_GP, RobotState.IDLE_NO_GP);
  private Set<RobotState> prepareStates =
      Set.of(
          RobotState.PREPARE_FLOOR_SHOT,
          RobotState.PREPARE_SUBWOOFER_SHOT,
          RobotState.PREPARE_SPEAKER_SHOT);

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
    var requirementsList = List.of(robot.intake, robot.queuer, robot.shooter, robot.climber);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command idleCommand() {
    return Commands.runOnce(robot::idleRequest, requirements)
        .andThen(robot.waitForStates(idleStates))
        .withName("IdleCommand");
  }

  public Command intakeCommand() {
    return Commands.runOnce(robot::intakeRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_W_GP))
        .withName("IntakeCommand");
  }

  public Command outtakeCommand() {
    return Commands.runOnce(robot::outtakeRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP))
        .withName("OuttakeCommand");
  }

  public Command outtakeShooterCommand() {
    return Commands.runOnce(robot::outtakeShooterRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP))
        .withName("OuttakeShooterCommand");
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, requirements)
        .andThen(robot.waitForStates(idleStates))
        .withName("UnjamCommand");
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
    return Commands.runOnce(robot::confirmShotRequest, requirements)
        .andThen(robot.waitForStates(prepareStates))
        .withName("ConfirmShotCommand");
  }

  public Command speakerShotCommand() {
    return Commands.runOnce(robot::speakerShotRequest, requirements).withName("SpeakerShotCommand");
  }

  public Command subwooferShotCommand() {
    return Commands.runOnce(robot::subwooferShotRequest, requirements).withName("SubwooferShotCommand");
  }

  public Command climbSequenceCommand() {
    return Commands.runOnce(robot::doClimbSequenceRequest, requirements)
        .andThen(
            robot.waitForStates(
                Set.of(
                    RobotState.CLIMBED,
                    RobotState.CLIMBING,
                    RobotState.WAITING_CLIMB,
                    RobotState.IDLE_NO_GP,
                    RobotState.IDLE_W_GP)))
        .withName("ClimbSequenceCommand");
  }

  public Command reverseClimbSequenceCommand() {
    return Commands.runOnce(robot::reverseClimbSequenceRequest, requirements)
        .andThen(
            robot.waitForStates(
                Set.of(
                    RobotState.CLIMBED,
                    RobotState.WAITING_CLIMB,
                    RobotState.IDLE_NO_GP,
                    RobotState.IDLE_W_GP)))
        .withName("ClimbSequenceCommand");
  }

  public Command homeClimberCommand() {
    return Commands.runOnce(robot::homeClimberRequest, requirements).withName("HomeClimberCommand");
  }

  public Command preloadNoteCommand() {
    return Commands.runOnce(robot::preloadNoteRequest, requirements).withName("preloadNoteCommand");
  }
}
