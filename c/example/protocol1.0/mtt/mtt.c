/*******************************************************************************
* Copyright 2018 DASAN Info Tech.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: SangYoung Lee */

//
// *********     MTT application      *********
//
//
// Available DXL model on this example : All models using Protocol 1.0
// This example is designed for using a Dynamixel MX-28, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <stdarg.h>
#include <float.h>
#include <math.h>
#include <sys/time.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

#define VERSION "1.00"

const char *program_name;   /* the name by which we were called */

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_TORQUE_LIMIT            34
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
//#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
//#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MINIMUM_POSITION_VALUE      1536                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      2560                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
//#define DXL_MOVING_STATUS_THRESHOLD     5                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

//#define MOTOR_ID                          1                   // Dynamixel ID: 1
uint8_t MOTOR_ID = 1;
//#define TORQUE_LIMIT                    100                 // Value of torque limit
uint16_t TORQUE_LIMIT = 200; //100;
//GOAL_ANGLE_DEGREE
float GOAL_ANGLE_DEGREE = 0.0;
//#define GOAL_TIME_OUT 1
int GOAL_TIME_OUT = 1; // in seconds

//#define PORT_NAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
//char *PORT_NAME = "/dev/ttyUSB0";
char *PORT_NAME = "/dev/ttyS4";
int PORT_SPEC = 485; // 232, 422, 485
//#define BAUD_RATE                        57600
int BAUD_RATE = 57600;

int disable_motor_torque = false;
int enable_motor_torque = false;
int disable_display_status = false;
int enable_goal_position = false;

typedef enum {
    STR2INT_SUCCESS,
    STR2INT_OVERFLOW,
    STR2INT_UNDERFLOW,
    STR2INT_INCONVERTIBLE
} str2int_errno;

str2int_errno str2int(int *out, char *s, int base)
{
    char *end;
    if (s[0] == '\0' || isspace(s[0]))
        return STR2INT_INCONVERTIBLE;
    errno = 0;
    long l = strtol(s, &end, base);
    /* Both checks are needed because INT_MAX == LONG_MAX is possible. */
    if (l > INT_MAX || (errno == ERANGE && l == LONG_MAX))
        return STR2INT_OVERFLOW;
    if (l < INT_MIN || (errno == ERANGE && l == LONG_MIN))
        return STR2INT_UNDERFLOW;
    if (*end != '\0')
        return STR2INT_INCONVERTIBLE;
    *out = l;
    return STR2INT_SUCCESS;
}

typedef enum {
    STR2FLOAT_SUCCESS,
    STR2FLOAT_OVERFLOW,
    STR2FLOAT_UNDERFLOW,
    STR2FLOAT_INCONVERTIBLE
} str2float_errno;

str2float_errno str2float(float *out, char *s)
{
    char *end;

    //printf("s=%p\n", s);
    //printf("s=%s\n", s);

    if (s[0] == '\0' || isspace(s[0]))
        return STR2FLOAT_INCONVERTIBLE;
    errno = 0;
    double l = strtof(s, &end);
    //printf("l=%f\n", l);
    //printf("end=%p\n", end);
    //printf("errno=%d\n", errno);
    //printf("FLT_MAX=%f\n", FLT_MAX);
    //printf("FLT_MIN=%f\n", FLT_MIN);
    //printf("DBL_MAX=%f\n", DBL_MAX);
    //printf("DBL_MIN=%f\n", DBL_MIN);
    /* Both checks are needed because FLT_MAX == DBL_MAX is possible. */
    if (l > FLT_MAX || (errno == ERANGE && l == DBL_MAX))
        return STR2FLOAT_OVERFLOW;
    if (l < FLT_MAX*-1. || (errno == ERANGE && l == DBL_MAX*-1.))
        return STR2FLOAT_UNDERFLOW;
    if (*end != '\0')
        return STR2FLOAT_INCONVERTIBLE;
    *out = l;
    return STR2FLOAT_SUCCESS;
}

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

static void
chkinvok(const char *s)
{
  const char *p;

  p = s;
  while (*p == '-')
    s = ++p;
  while (*p)
    if (*p++ == '/')
      s = p;
  program_name = s;
}

static void
usage(int exitcode, const char *format, ...)
{
  FILE *f=stdout;

  if (exitcode)
  {
    if (format)
    {
      fprintf(stderr, "%s: ",program_name);

      va_list arg;
      va_start(arg, format);
      vfprintf(stderr, format, arg);
      va_end(arg);

      fprintf(stderr, "\n");
    }
    fprintf (stderr, "Try `%s -h' for more information.\n", program_name);
    exit(exitcode);
  }

  fprintf(f, "%s version %s\n", program_name, VERSION);

  fprintf(f, "Usage: %s [GOAL_ANGLE_DEGREE] [-i:l:t:dexn:s:b:h]\n", program_name);
  fputs("Moter test tool\n", f);
  fputs((
"  GOAL_ANGLE_DEGREE : move position to GOAL_ANGLE_DEGREE with enable motor torque, -50.0~50.0\n"
"  -i MOTOR_ID       : use motor ID to MOTOR_ID, 1~255, default=1\n"
"  -l TORQUE_LIMIT   : set motor torque limit to TORQUE_LIMIT, 0~1023, default=200\n"
"  -t GOAL_TIME_OUT  : time out value in seconds to move goal position, 1~100, default=1\n"
"  -d                : disable moter torque\n"
"  -e                : enable moter torque\n"
"  -x                : disable display status\n"
"  -n PORT_NAME      : set serial port name to PORT_NAME, default=/dev/ttyS4\n"
"  -s PORT_SPEC      : set serial port spec to PORT_SPEC, 232,422,485, default=485\n"
"  -b BAUD_RATE      : set serial baud rate to BAUD_RATE, default=57600\n"
"  -h                : display usage\n"
    ), f);
  exit(exitcode);
}

uint16_t angle_degree_2_position(float angle_degree)
{
  return (uint16_t)(truncf((180. + angle_degree) * 4096. / 360. + 0.5));
}

float position_2_angle_degree(uint16_t position)
{
  return ((float)position * 360. / 4096.) - 180.;
}

int ping_id(int port_num)
{
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_model_number;                      // Dynamixel model number

  // Try to ping the Dynamixel
  // Get Dynamixel model number
  dxl_model_number = pingGetModelNum(port_num, PROTOCOL_VERSION, MOTOR_ID);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    return -1;
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    return -1;
  }

  //printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", MOTOR_ID, dxl_model_number);

  return 0;
}

void set_goal_position(int port_num)
{
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint16_t dxl_goal_position = angle_degree_2_position(GOAL_ANGLE_DEGREE);  // Goal position
  uint8_t dxl_error;                          // Dynamixel error
  uint16_t dxl_present_position;              // Present position
  uint16_t dxl_present_position_prev;         // Present position

  //printf("[ID:%03d] dxl_goal_position:%d\n", MOTOR_ID, dxl_goal_position);

  // Set Torque limit
  write2ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
  else
  {
    //printf("Dynamixel has been successfully connected \n");
  }

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
  else
  {
    //printf("Dynamixel has been successfully connected \n");
  }

  //printf("Press any key to continue! (or press ESC to quit!)\n");
  //if (getch() == ESC_ASCII_VALUE)
  //  break;

  // Write goal position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }

  struct timeval time_start, time_end;

  dxl_present_position_prev = 0xffff;
  do
  {
    // Read present position
    dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_PRESENT_POSITION);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
    //printf("ID:%d GP:%d:%02.1f PP:%d:%02.1f\n", MOTOR_ID, dxl_goal_position, position_2_angle_degree(dxl_goal_position), dxl_present_position, position_2_angle_degree(dxl_present_position));
    if (dxl_present_position_prev == 0xffff)
    {
      // start timer
      gettimeofday(&time_start, NULL);
      dxl_present_position_prev = dxl_present_position;
    }
    else
    {
      if (abs(dxl_present_position - dxl_present_position_prev) > 1)
      {
        gettimeofday(&time_start, NULL);
      }
      dxl_present_position_prev = dxl_present_position;
    }
    // end timer
    gettimeofday(&time_end, NULL);
    if (((time_end.tv_sec - time_start.tv_sec) * 1000 + (time_end.tv_usec - time_start.tv_usec) / 1000) >= GOAL_TIME_OUT * 1000)
    {
      break;
    }
    usleep(100*1000); // sleep 100ms
  } while ((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

  //printf("ID:%d TI:%ldms\n", MOTOR_ID, ((time_end.tv_sec - time_start.tv_sec) * 1000 + (time_end.tv_usec - time_start.tv_usec) / 1000));
  //printf("ID:%d GP:%d:%02.1f PP:%d:%02.1f\n", MOTOR_ID, dxl_goal_position, position_2_angle_degree(dxl_goal_position), dxl_present_position, position_2_angle_degree(dxl_present_position));
}

void set_enable_motor_torque(int port_num)
{
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
}

void set_disable_motor_torque(int port_num)
{
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
}

void show_status(int port_num)
{
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t goal_position;
  uint16_t present_position;
  uint16_t torque_limit;
  uint8_t torque_enable;

  // Read goal position
  goal_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_GOAL_POSITION);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }

  // Read present position
  present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_PRESENT_POSITION);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }

  // Read Torque limit
  torque_limit = read2ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_TORQUE_LIMIT);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
  //printf("[ID:%03d] Toque limit:%d\n", MOTOR_ID, torque_limit);

  // Read Torque Enable
  torque_enable = read1ByteTxRx(port_num, PROTOCOL_VERSION, MOTOR_ID, ADDR_MX_TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
  //printf("[ID:%03d] Toque enable:%d\n", MOTOR_ID, torque_enable);

  printf("ID:%d GP:%d:%.1f PP:%d:%.1f TL:%d TE:%d\n", MOTOR_ID, goal_position, position_2_angle_degree(goal_position), present_position, position_2_angle_degree(present_position), torque_limit, torque_enable);
}

int main(int argc, char **argv)
{
  int index;
  int c;
  int ival;
  float fval;
  int skip_getopt = false;

  chkinvok(argv[0]);

  //printf("argc:%d\n", argc);

  if (argc > 1 && (isdigit(argv[1][0]) || (argv[1][0] == '-' && isdigit(argv[1][1]))))
  {
    //printf("argv[1]:%s\n", argv[1]);
    if (str2float(&fval, argv[1]) != STR2FLOAT_SUCCESS)
    {
      usage(-1, "incorrct argument of %s.\n", argv[1]);
      return -1;
    }
    if (fval < -50.0 || fval > 50.0)
    {
      usage(-1, "GOAL_ANGLE_DEGREE must be -50.0~50.0.\n");
      return -1;
    }
    GOAL_ANGLE_DEGREE = fval;
    enable_goal_position = true;
    enable_motor_torque = true;
    //printf("GOAL_ANGLE_DEGREE:%f\n", GOAL_ANGLE_DEGREE);
    if (argc > 2)
    {
      optind ++; // skip first arg for getopt
    }
    else
    {
      skip_getopt = true;
    }
  }

  if (skip_getopt == false)
  {
    while ((c = getopt (argc, argv, "i:l:t:dexn:s:b:h")) != -1)
    {
      switch (c)
      {
        case 'i':
          if (str2int(&ival, optarg, 10) != STR2INT_SUCCESS)
          {
            usage(-1, "incorrct argument of option -%c.\n", optopt);
            break;
          }
          if (ival < 1 || ival > 255)
          {
            usage(-1, "MOTOR_ID must be 1~255.\n");
            break;
          }
          MOTOR_ID = (uint8_t)ival;
          break;
        case 'l':
          if (str2int(&ival, optarg, 10) != STR2INT_SUCCESS)
          {
            usage(-1, "incorrct argument of option -%c.\n", optopt);
            break;
          }
          if (ival < 0 || ival > 1023)
          {
            usage(-1, "TORQUE_LIMIT must be 0~1023.\n");
            break;
          }
          TORQUE_LIMIT = (uint16_t)ival;
          break;
        case 't':
          if (str2int(&ival, optarg, 10) != STR2INT_SUCCESS)
          {
            usage(-1, "incorrct argument of option -%c.\n", optopt);
            break;
          }
          if (ival < 1 || ival > 100)
          {
            usage(-1, "GOAL_TIME_OUT must be 1~100.\n");
            break;
          }
          GOAL_TIME_OUT = ival;
          break;
        case 'd':
          disable_motor_torque = true;
          enable_motor_torque = false;
          break;
        case 'e':
          enable_motor_torque = true;
          disable_motor_torque = false;
          break;
        case 'x':
          disable_display_status = true;
          break;
        case 'n':
          PORT_NAME = optarg;
          break;
        case 's':
          if (str2int(&ival, optarg, 10) != STR2INT_SUCCESS)
          {
            usage(-1, "incorrct argument of option -%c.\n", optopt);
            break;
          }
          if (ival != 232 && ival != 422 && ival != 485)
          {
            usage(-1, "PORT_SPEC must be 232, 422 or 485.\n");
            break;
          }
          PORT_SPEC = ival;
          break;
        case 'b':
          if (str2int(&ival, optarg, 10) != STR2INT_SUCCESS)
          {
            usage(-1, "incorrct argument of option -%c.\n", optopt);
            break;
          }
          switch (ival)
          {
            case 57600:
            case 115200:
              break;
            default:
              usage(-1, "BAUD_RATE must be 57600, 115200.\n");
              break;
          }
          BAUD_RATE = ival;
          break;
        case 'h':
          usage(0, NULL);
          break;
        case '?':
          switch (optopt)
          {
            case 'l':
            case 'n':
            case 'b':
              usage(-1, NULL);
              break;
          }
          if (isprint (optopt))
            usage(-1, NULL);
          else
            usage(-1, "Unknown option character `\\x%x'.\n", optopt);
          return -1;
        default:
          abort ();
      }
    }

    for (index = optind; index < argc; index++)
    {
      printf ("Non-option argument %s\n", argv[index]);
    }
  }

  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(PORT_NAME);

  // Initialize PacketHandler Structs
  packetHandler();

  // Open port
  if (!openPort(port_num))
  {
    usage(-1, "Failed to open the port! %s\n", PORT_NAME);
    return 0;
  }

  // Set port baudrate
  if (!setBaudRate(port_num, BAUD_RATE))
  {
    // Close port
    closePort(port_num);
    usage(-1, "Failed to change the baudrate! %d\n", BAUD_RATE);
    return 0;
  }

  if (ping_id(port_num) == 0)
  {
    if (enable_goal_position)
    {
      set_goal_position(port_num);
      enable_motor_torque = false; // Already enabled.
    }

    if (enable_motor_torque)
      set_enable_motor_torque(port_num);

    if (disable_motor_torque)
      set_disable_motor_torque(port_num);

    if (!disable_display_status)
      show_status(port_num);
  }

  // Close port
  closePort(port_num);

  return 0;
}

