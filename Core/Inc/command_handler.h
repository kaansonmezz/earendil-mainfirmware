#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "terminal_parser.h"

/* ── Public API ───────────────────────────────────────────────────────────── */
void CommandHandler_Handle(const TerminalCommand_t *cmd);
void CommandHandler_PrintHelp(void);

#endif /* COMMAND_HANDLER_H */
