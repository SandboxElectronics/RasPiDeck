/*
*  Copyright (C) 2012 Libelium Comunicaciones Distribuidas S.L.
*  http://www.libelium.com
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Version 1.5 (For Raspberry Pi Rev2)
*  Author: Anartz Nuin Jiménez
*/

#ifndef INCLUDES_HEADER
#define INCLUDES_HEADER

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <termios.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <algorithm>
#include <limits.h>
#include <pthread.h>
#include <poll.h>

#include "cores/wiring.h"
#include "Wire/Wire.h"
#include "Peripherals/i2cio.h"
#include "Peripherals/i2cio16.h"
#include "Peripherals/Peripherals.h"

#define BIT_0  (1 <<  0)
#define BIT_1  (1 <<  1)
#define BIT_2  (1 <<  2)
#define BIT_3  (1 <<  3)
#define BIT_4  (1 <<  4)
#define BIT_5  (1 <<  5)
#define BIT_6  (1 <<  6)
#define BIT_7  (1 <<  7)
#define BIT_8  (1 <<  8)
#define BIT_9  (1 <<  9)
#define BIT_10 (1 << 10)
#define BIT_11 (1 << 11)
#define BIT_12 (1 << 12)
#define BIT_13 (1 << 13)
#define BIT_14 (1 << 14)
#define BIT_15 (1 << 15)
#define BIT_16 (1 << 16)
#define BIT_17 (1 << 17)
#define BIT_18 (1 << 18)
#define BIT_19 (1 << 19)
#define BIT_20 (1 << 20)
#define BIT_21 (1 << 21)
#define BIT_22 (1 << 22)
#define BIT_23 (1 << 23)
#define BIT_24 (1 << 24)
#define BIT_25 (1 << 25)
#define BIT_26 (1 << 26)
#define BIT_27 (1 << 27)
#define BIT_28 (1 << 28)
#define BIT_29 (1 << 29)
#define BIT_30 (1 << 30)
#define BIT_31 (1 << 31)

/*
 * Arduino Pin Definition
 */

#define SS   10
#define MOSI 11
#define MISO 12
#define SCK  13

#define SDA  18
#define SCL  19

#define A0   14
#define A1   15
#define A2   16
#define A3   17
#define A4   18
#define A5   19
#define A6   20
#define A7   21
#define PL  (0xFF)

#endif // INCLUDES_HEADER
