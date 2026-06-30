#ifndef TERMINAL_IF_H
#define TERMINAL_IF_H

#include "rover_types.h"
#include <stddef.h>

/* ── Public API ───────────────────────────────────────────────────────────
 * Terminal (USART3) input module.
 * Accumulates incoming bytes into lines and queues completed lines in a FIFO
 * so a burst of commands from the GUI cannot overwrite an unprocessed line.
 * The main loop drains the queue via TerminalIf_GetLine().
 * ─────────────────────────────────────────────────────────────────────────── */

void    TerminalIf_Init(void);
void    TerminalIf_Process(void);
uint8_t TerminalIf_RxCallback(uint8_t byte);

/* Pop one completed line from the FIFO into `outLine`.
 * Returns true if a line was returned, false if the queue is empty. */
bool    TerminalIf_GetLine(char *outLine, size_t outSize);

/* Queue diagnostics (safe to call from main-loop context). */
uint8_t  TerminalIf_GetPendingLineCount(void);
uint32_t TerminalIf_GetDroppedLineCount(void);
uint32_t TerminalIf_GetReceivedLineCount(void);
uint8_t  TerminalIf_GetMaxLineQueueDepth(void);

/* ── Legacy single-line API (deprecated) ──────────────────────────────────
 * Prefer TerminalIf_GetLine().  Kept for backward compatibility. */
bool        TerminalIf_LineReady(void);
const char *TerminalIf_GetLinePtr(void);

#endif /* TERMINAL_IF_H */