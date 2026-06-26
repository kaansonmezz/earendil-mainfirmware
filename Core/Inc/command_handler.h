#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "terminal_parser.h"

/* ── Public API ───────────────────────────────────────────────────────────── */
void CommandHandler_Handle(const TerminalCommand_t *cmd);
void CommandHandler_PrintHelp(void);

/* ── Command classification (used by the DISARM gate) ─────────────────────── */
bool Command_IsModeTransition(const TerminalCommand_t *cmd);
bool Command_IsMotionCommand(const TerminalCommand_t *cmd);

#endif /* COMMAND_HANDLER_H */
