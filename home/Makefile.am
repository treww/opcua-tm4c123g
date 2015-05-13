# Author: Alexander Rykovanov 2012
#
# Distributed under the GNU LGPL License
# (See accompanying file LICENSE or copy at 
# http://www.gnu.org/copyleft/gpl.html)
#
# $Id:  $
# $Date: $
# $Revision: $

AUTOMAKE_OPTIONS=subdir-objects

#######################################################
# list of binaries.
#######################################################

bin_PROGRAMS = freertos_home

####################################################
# server executable
####################################################

FREERTOS_POSIX_ROOT=FreeRTOS/FreeRTOS/Source
FREERTOS_POSIX_PORTABLE=FreeRTOS/FreeRTOS/Source/portable/MSVC-MingW

freertos_home_SOURCES = \
	main.c \
	led_task.c \
	led_task.h \
	control.c \
	control.h \
	FreeRTOS/FreeRTOS/Source/queue.c \
	FreeRTOS/FreeRTOS/Source/tasks.c \
	FreeRTOS/FreeRTOS/Source/list.c \
	FreeRTOS/FreeRTOS/Source/portable/MSVC-MingW/port.c \
	FreeRTOS/FreeRTOS/Source/portable/MemMang/heap_3.c


freertos_home_CFLAGS = -pthread -I. -I$(FREERTOS_POSIX_ROOT)/include -I$(FREERTOS_POSIX_PORTABLE) -I$(srcdir)/../../ti/inc
freertos_home_LDFLAGS = -ldl


target:
	$(MAKE) -f Makefile.mcu

clean-local:
	$(MAKE) -f Makefile.mcu clean

flash: target
	sudo lm4flash gcc/freertos_home.bin

.PHONY: target flash

