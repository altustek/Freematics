#ifndef ATPIPE_H_INCLUDED
#define ATPIPE_H_INCLUDED

// Wait timeoutSec seconds for a key on the USB serial port. If one comes,
// bridge the serial port to the OBD co-processor or the cellular modem to
// type AT commands by hand:
//   Ctrl+N  switch module    Ctrl+P  toggle modem power
//   Ctrl+E  toggle echo      Ctrl+D  end (restarts if anything was sent)
// Must run before sys.begin(), which takes over the UARTs.
void atPipe(unsigned int timeoutSec);

#endif
