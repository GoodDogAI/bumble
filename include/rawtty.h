#include <termios.h> // Contains POSIX terminal control definitions

#ifndef RAW_SERIAL_H
#define RAW_SERIAL_H

// This function puts a Linux TTY serial port connection into "raw mode"
// Meaning, no echoing behavior, no weird special characters, just
// sending raw data to a serial device
void tty_raw(struct termios *raw) {
  /* input modes - clear indicated ones giving: no break, no CR to NL, 
      no parity check, no strip char, no start/stop output (sic) control */
  raw->c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  /* output modes - clear giving: no post processing such as NL to CR+NL */
  raw->c_oflag &= ~(OPOST);

  /* control modes - set 8 bit chars */
  raw->c_cflag |= (CS8);

  /* local modes - clear giving: echoing off, canonical off (no erase with 
      backspace, ^U,...),  no extended functions, no signal chars (^Z,^C) */
  raw->c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

  /* control chars - set return condition: min number of bytes and timer */
  // raw->c_cc[VMIN] = 5; raw->c_cc[VTIME] = 8; /* after 5 bytes or .8 seconds
  //                                             after first byte seen      */
  raw->c_cc[VMIN] = 0; raw->c_cc[VTIME] = 0; /* immediate - anything       */
  // raw->c_cc[VMIN] = 2; raw->c_cc[VTIME] = 0; /* after two bytes, no timer  */
  // raw->c_cc[VMIN] = 0; raw->c_cc[VTIME] = 8; /* after a byte or .8 seconds */
}

#endif