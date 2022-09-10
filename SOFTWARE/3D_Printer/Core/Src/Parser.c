#include "Parser.h"

void reset_args()
{
  R = -1;
  T = 0;
  S = 0;
}

void get_command(uint8_t buf[])
{
  char *command;
  char *delim = " ";

  split_gcode(buf);
  command = strtok(buf, delim);
  switch (command[0])
  {
  case 'M':
    HAL_Delay(1);
    switch (atoi(buf + 1))
    {
    case 105:
      M105(R, T);
      break;
    case 155:
      interval = M155(S);
      break;
    }
    break;
  }
  reset_args();
  memset(&command[0], 0, sizeof(command));
}

void split_gcode(uint8_t buf[])
{
  char *buf2;
  buf2 = strtok(buf, " ");
  while (buf2 != NULL)
  {

    switch (buf2[0])
    {
    case 'R':
      R = atoi(buf2 + 1);
      break;
    case 'T':
      T = atoi(buf2 + 1);
      break;
    case 'S':
      S = atoi(buf2 + 1);
      break;
    }
    buf2 = strtok(NULL, " ");
  }
}