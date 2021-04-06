/****************************************************************************

  Copyright (c) 2021 Ronald Sieber

  Project:      Generic / Project independent
  Description:  Implementation of DEBUG TRACE

  -------------------------------------------------------------------------

  Revision History:

  2021/01/22 -rs:   V1.00 Initial version

****************************************************************************/


#include <stdio.h>
#include <stdarg.h>
#include "Arduino.h"



//---------------------------------------------------------------------------
// trace
//---------------------------------------------------------------------------

#define     BASIC_DELAY     1                   // Basic delay [ms]

void  trace (const char* pszFmt_p, ...)
{

char     szBuffer[0x400];
va_list  pArgList;


    // assemble message to output
    va_start (pArgList, pszFmt_p);
    vsprintf (szBuffer, pszFmt_p, pArgList);
    va_end   (pArgList);

    // output message to serial interface
    Serial.print(szBuffer);

    // wait until serial buffer is empty
    Serial.flush();
    delay(BASIC_DELAY);


    return;

}



// EOF
