package frc.robot.util;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.BooleanSupplier;

public class ContinuouslyConditionalCommand extends Command {
    private final Command m_onTrue;
    private final Command m_onFalse;
    private final BooleanSupplier m_condition;
    private boolean m_negated;

    public ContinuouslyConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.m_onTrue = ErrorMessages.requireNonNullParam(onTrue, "onTrue", "ConditionalCommand");
        this.m_onFalse = ErrorMessages.requireNonNullParam(onFalse, "onFalse", "ConditionalCommand");
        this.m_condition = ErrorMessages.requireNonNullParam(condition, "condition", "ConditionalCommand");
        CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);
        this.m_negated = condition.getAsBoolean();
    }

    private Command getCurrentCommand(){
        if (m_condition.getAsBoolean()) return m_onTrue;
        return m_onFalse;
    }

    private Command getOtherCommand(){
        if (m_condition.getAsBoolean()) return m_onFalse;
        return m_onTrue;
    }

    public void initialize() {
        getCurrentCommand().schedule();
    }

    public void execute() {
        if (m_negated != m_condition.getAsBoolean()) {
            getOtherCommand().cancel();
            getCurrentCommand().schedule();
            m_negated = !m_negated;
        }
    }
}
