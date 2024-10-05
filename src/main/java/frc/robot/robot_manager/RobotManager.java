package frc.robot.robot_manager;

import frc.robot.climber.ClimberSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.queuer.QueuerState;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class RobotManager extends StateMachine<RobotState> {
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final QueuerSubsystem queuer;
  private final ClimberSubsystem climber;

  // INIT
  public RobotManager(
      IntakeSubsystem intake,
      QueuerSubsystem queuer,
      ShooterSubsystem shooter,
      ClimberSubsystem climber) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.intake = intake;
    this.shooter = shooter;
    this.queuer = queuer;
    this.climber = climber;
  }
  // --------------------

  @Override
  protected void collectInputs() {}

  // Automatic state transitions
  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case WAITING_SUBWOOFER_SHOT,
              WAITING_FLOOR_SHOT,
              IDLE_NO_GP,
              IDLE_W_GP,
              WAITING_CLIMB,
              CLIMBING,
              CLIMBED,
              UNJAM ->
          currentState;
      case SUBWOOFER_SHOT, FLOOR_SHOT -> queuer.hasNote() ? currentState : RobotState.IDLE_NO_GP;

      case PREPARE_SUBWOOFER_SHOT ->
          shooter.atGoal(ShooterState.SUBWOOFER_SHOT) ? RobotState.SUBWOOFER_SHOT : currentState;

      case PREPARE_FLOOR_SHOT ->
          shooter.atGoal(ShooterState.FLOOR_SHOT) ? RobotState.FLOOR_SHOT : currentState;
      case INTAKING -> queuer.hasNote() ? RobotState.IDLE_W_GP : currentState;
      case OUTTAKING -> queuer.hasNote() ? currentState : RobotState.IDLE_NO_GP;
    };
  }
  // ---------------------

  // State actions
  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
        case IDLE_NO_GP -> {
            shooter.setState(ShooterState.STOPPED);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(false);
        }
        
        case IDLE_W_GP -> {
            shooter.setState(ShooterState.IDLE);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(false);
        }
        case INTAKING -> {
            shooter.setState(ShooterState.STOPPED);
            intake.setState(IntakeState.INTAKE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(false);
        }
        case OUTTAKING -> {
            shooter.setState(ShooterState.IDLE);
            intake.setState(IntakeState.OUTTAKE);
            queuer.setState(QueuerState.TO_INTAKE);
            climber.setRaised(false);
        }
        case WAITING_SUBWOOFER_SHOT -> {
            shooter.setState(ShooterState.SUBWOOFER_SHOT);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(false);
        }
        case WAITING_FLOOR_SHOT -> {
            shooter.setState(ShooterState.FLOOR_SHOT);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(false);
        }
        case PREPARE_SUBWOOFER_SHOT -> {
            shooter.setState(ShooterState.SUBWOOFER_SHOT);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(false);
        }
        case PREPARE_FLOOR_SHOT -> {
            shooter.setState(ShooterState.FLOOR_SHOT);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(false);
        }
        case FLOOR_SHOT -> {
            shooter.setState(ShooterState.FLOOR_SHOT);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.TO_SHOOTER);
            climber.setRaised(false);
        }
        case SUBWOOFER_SHOT -> {
            shooter.setState(ShooterState.SUBWOOFER_SHOT);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.TO_SHOOTER);
            climber.setRaised(false);
        }
        case UNJAM -> {
            shooter.setState(ShooterState.IDLE);
            intake.setState(IntakeState.OUTTAKE);
            queuer.setState(QueuerState.TO_SHOOTER);
            climber.setRaised(false);
        }
        case WAITING_CLIMB -> {
            shooter.setState(ShooterState.STOPPED);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(true);
        }
        case CLIMBING -> {
            shooter.setState(ShooterState.STOPPED);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(false);
        }
        case CLIMBED -> {
            shooter.setState(ShooterState.STOPPED);
            intake.setState(IntakeState.IDLE);
            queuer.setState(QueuerState.IDLE);
            climber.setRaised(false);
        }

    }
  }
}
