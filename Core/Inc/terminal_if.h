#ifndef TERMINAL_IF_H
#define TERMINAL_IF_H

#include "rover_types.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
void    TerminalIf_Init(void);
void    TerminalIf_Process(void);
uint8_t TerminalIf_RxCallback(uint8_t byte);
bool    TerminalIf_LineReady(void);
const char *TerminalIf_GetLine(void);

#endif /* TERMINAL_IF_H */
