/* Copyright (c) 2002,2003,2005,2006,2007 Marek Michalkiewicz, Joerg Wunsch
   Copyright (c) 2007 Eric B. Weddington
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

/* $Id: io.h,v 1.52.2.19 2009/02/19 21:10:54 arcanum Exp $ */

/** \file */
/** \defgroup avr_io : AVR device-specific IO definitions
    \code #include  \endcode

    This header file includes the apropriate IO definitions for the
    device that has been specified by the -mmcu= compiler
    command-line switch.  This is done by diverting to the appropriate
    file <avr/ioXXXX.h> which should
    never be included directly.  Some register names common to all
    AVR devices are defined directly within <avr/common.h>,
    which is included in <avr/io.h>,
    but most of the details come from the respective include file.

    Note that this file always includes the following files:
    \code 
    #include 
    #include 
    #include 
    #include 
    \endcode
    See \ref avr_sfr for more details about that header file.

    Included are definitions of the IO register set and their
    respective bit values as specified in the Atmel documentation.
    Note that inconsistencies in naming conventions,
    so even identical functions sometimes get different names on
    different devices.

    Also included are the specific names useable for interrupt
    function definitions as documented
    \ref avr_signames "here".

    Finally, the following macros are defined:

    - \b RAMEND
    

    The last on-chip RAM address.
    

    - \b XRAMEND
    

    The last possible RAM location that is addressable. This is equal to 
    RAMEND for devices that do not allow for external RAM. For devices 
    that allow external RAM, this will be larger than RAMEND.
    

    - \b E2END
    

    The last EEPROM address.
    

    - \b FLASHEND
    

    The last byte address in the Flash program space.
    

    - \b SPM_PAGESIZE
    

    For devices with bootloader support, the flash pagesize
    (in bytes) to be used for the \c SPM instruction. 
    - \b E2PAGESIZE
    

    The size of the EEPROM page.
    
*/

#ifndef _AVR_IO_H_
#define _AVR_IO_H_

#include 

#if defined (__AVR_AT94K__)
#  include 
#elif defined (__AVR_AT43USB320__)
#  include 
#elif defined (__AVR_AT43USB355__)
#  include 
#elif defined (__AVR_AT76C711__)
#  include 
#elif defined (__AVR_AT86RF401__)
#  include 
#elif defined (__AVR_AT90PWM1__)
#  include 
#elif defined (__AVR_AT90PWM2__)
#  include 
#elif defined (__AVR_AT90PWM2B__)
#  include 
#elif defined (__AVR_AT90PWM3__)
#  include 
#elif defined (__AVR_AT90PWM3B__)
#  include 
#elif defined (__AVR_AT90PWM216__)
#  include 
#elif defined (__AVR_AT90PWM316__)
#  include 
#elif defined (__AVR_AT90PWM81__)
#  include 
#elif defined (__AVR_ATmega16M1__)
#  include 
#elif defined (__AVR_ATmega16U4__)
#  include 
#elif defined (__AVR_ATmega32C1__)
#  include 
#elif defined (__AVR_ATmega32M1__)
#  include 
#elif defined (__AVR_ATmega32U4__)
#  include 
#elif defined (__AVR_ATmega32U6__)
#  include 
#elif defined (__AVR_ATmega64C1__)
#  include 
#elif defined (__AVR_ATmega64M1__)
#  include 
#elif defined (__AVR_ATmega128__)
#  include 
#elif defined (__AVR_ATmega1280__)
#  include 
#elif defined (__AVR_ATmega1281__)
#  include 
#elif defined (__AVR_ATmega1284P__)
#  include 
#elif defined (__AVR_ATmega128RFA1__)
#  include 
#elif defined (__AVR_ATmega2560__)
#  include 
#elif defined (__AVR_ATmega2561__)
#  include 
#elif defined (__AVR_AT90CAN32__)
#  include 
#elif defined (__AVR_AT90CAN64__)
#  include 
#elif defined (__AVR_AT90CAN128__)
#  include 
#elif defined (__AVR_AT90USB82__)
#  include 
#elif defined (__AVR_AT90USB162__)
#  include 
#elif defined (__AVR_AT90USB646__)
#  include 
#elif defined (__AVR_AT90USB647__)
#  include 
#elif defined (__AVR_AT90USB1286__)
#  include 
#elif defined (__AVR_AT90USB1287__)
#  include 
#elif defined (__AVR_ATmega64__)
#  include 
#elif defined (__AVR_ATmega640__)
#  include 
#elif defined (__AVR_ATmega644__)
#  include 
#elif defined (__AVR_ATmega644P__)
#  include 
#elif defined (__AVR_ATmega645__)
#  include 
#elif defined (__AVR_ATmega6450__)
#  include 
#elif defined (__AVR_ATmega649__)
#  include 
#elif defined (__AVR_ATmega6490__)
#  include 
#elif defined (__AVR_ATmega103__)
#  include 
#elif defined (__AVR_ATmega32__)
#  include 
#elif defined (__AVR_ATmega323__)
#  include 
#elif defined (__AVR_ATmega324P__)
#  include 
#elif defined (__AVR_ATmega325__)
#  include 
#elif defined (__AVR_ATmega325P__)
#  include 
#elif defined (__AVR_ATmega3250__)
#  include 
#elif defined (__AVR_ATmega3250P__)
#  include 
#elif defined (__AVR_ATmega328P__)
#  include 
#elif defined (__AVR_ATmega329__)
#  include 
#elif defined (__AVR_ATmega329P__)
#  include 
#elif defined (__AVR_ATmega3290__)
#  include 
#elif defined (__AVR_ATmega3290P__)
#  include 
#elif defined (__AVR_ATmega32HVB__)
#  include 
#elif defined (__AVR_ATmega406__)
#  include 
#elif defined (__AVR_ATmega16__)
#  include 
#elif defined (__AVR_ATmega161__)
#  include 
#elif defined (__AVR_ATmega162__)
#  include 
#elif defined (__AVR_ATmega163__)
#  include 
#elif defined (__AVR_ATmega164P__)
#  include 
#elif defined (__AVR_ATmega165__)
#  include 
#elif defined (__AVR_ATmega165P__)
#  include 
#elif defined (__AVR_ATmega168__)
#  include 
#elif defined (__AVR_ATmega168P__)
#  include 
#elif defined (__AVR_ATmega169__)
#  include 
#elif defined (__AVR_ATmega169P__)
#  include 
#elif defined (__AVR_ATmega8HVA__)
#  include 
#elif defined (__AVR_ATmega16HVA__)
#  include 
#elif defined (__AVR_ATmega8__)
#  include 
#elif defined (__AVR_ATmega48__)
#  include 
#elif defined (__AVR_ATmega48P__)
#  include 
#elif defined (__AVR_ATmega88__)
#  include 
#elif defined (__AVR_ATmega88P__)
#  include 
#elif defined (__AVR_ATmega8515__)
#  include 
#elif defined (__AVR_ATmega8535__)
#  include 
#elif defined (__AVR_AT90S8535__)
#  include 
#elif defined (__AVR_AT90C8534__)
#  include 
#elif defined (__AVR_AT90S8515__)
#  include 
#elif defined (__AVR_AT90S4434__)
#  include 
#elif defined (__AVR_AT90S4433__)
#  include 
#elif defined (__AVR_AT90S4414__)
#  include 
#elif defined (__AVR_ATtiny22__)
#  include 
#elif defined (__AVR_ATtiny26__)
#  include 
#elif defined (__AVR_AT90S2343__)
#  include 
#elif defined (__AVR_AT90S2333__)
#  include 
#elif defined (__AVR_AT90S2323__)
#  include 
#elif defined (__AVR_AT90S2313__)
#  include 
#elif defined (__AVR_ATtiny2313__)
#  include 
#elif defined (__AVR_ATtiny13__)
#  include 
#elif defined (__AVR_ATtiny13A__)
#  include 
#elif defined (__AVR_ATtiny25__)
#  include 
#elif defined (__AVR_ATtiny45__)
#  include 
#elif defined (__AVR_ATtiny85__)
#  include 
#elif defined (__AVR_ATtiny24__)
#  include 
#elif defined (__AVR_ATtiny44__)
#  include 
#elif defined (__AVR_ATtiny84__)
#  include 
#elif defined (__AVR_ATtiny261__)
#  include 
#elif defined (__AVR_ATtiny461__)
#  include 
#elif defined (__AVR_ATtiny861__)
#  include 
#elif defined (__AVR_ATtiny43U__)
#  include 
#elif defined (__AVR_ATtiny48__)
#  include 
#elif defined (__AVR_ATtiny88__)
#  include 
#elif defined (__AVR_ATtiny87__)
#  include 
#elif defined (__AVR_ATtiny167__)
#  include 
#elif defined (__AVR_AT90SCR100__)
#  include 
#elif defined (__AVR_ATxmega16A4__)
#  include 
#elif defined (__AVR_ATxmega16D4__)
#  include 
#elif defined (__AVR_ATxmega32A4__)
#  include 
#elif defined (__AVR_ATxmega32D4__)
#  include 
#elif defined (__AVR_ATxmega64A1__)
#  include 
#elif defined (__AVR_ATxmega64A3__)
#  include 
#elif defined (__AVR_ATxmega128A1__)
#  include 
#elif defined (__AVR_ATxmega128A3__)
#  include 
#elif defined (__AVR_ATxmega256A3__)
#  include 
#elif defined (__AVR_ATxmega256A3B__)
#  include 
#elif defined (__AVR_ATA6289__)
#  include 
/* avr1: the following only supported for assembler programs */
#elif defined (__AVR_ATtiny28__)
#  include 
#elif defined (__AVR_AT90S1200__)
#  include 
#elif defined (__AVR_ATtiny15__)
#  include 
#elif defined (__AVR_ATtiny12__)
#  include 
#elif defined (__AVR_ATtiny11__)
#  include 
#else
#  if !defined(__COMPILING_AVR_LIBC__)
#    warning "device type not defined"
#  endif
#endif

#include 

#include 

#include 

/* Include fuse.h after individual IO header files. */
#include 

/* Include lock.h after individual IO header files. */
#include 

#endif /* _AVR_IO_H_ */
