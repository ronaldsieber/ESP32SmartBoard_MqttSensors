/****************************************************************************

  Copyright (c) 2021 Ronald Sieber

  Project:      Generic / Project independent
  Description:  Definition of DEBUG TRACE

  -------------------------------------------------------------------------

  Revision History:

  2021/01/22 -rs:   V1.00 Initial version

****************************************************************************/



//---------------------------------------------------------------------------
//  Definitions for TRACE Macro and Function
//---------------------------------------------------------------------------

#ifdef DEBUG

    #define TRACE  trace
    void  trace (const char* pszFmt_p, ...);

    #ifndef TRACE
        #define TRACE
    #endif

    #ifndef TRACE0
        #define TRACE0(p0)                              TRACE(p0)
    #endif

    #ifndef TRACE1
        #define TRACE1(p0, p1)                          TRACE(p0, p1)
    #endif

    #ifndef TRACE2
        #define TRACE2(p0, p1, p2)                      TRACE(p0, p1, p2)
    #endif

    #ifndef TRACE3
        #define TRACE3(p0, p1, p2, p3)                  TRACE(p0, p1, p2, p3)
    #endif

    #ifndef TRACE4
        #define TRACE4(p0, p1, p2, p3, p4)              TRACE(p0, p1, p2, p3, p4)
    #endif

    #ifndef TRACE5
        #define TRACE5(p0, p1, p2, p3, p4, p5)          TRACE(p0, p1, p2, p3, p4, p5)
    #endif

    #ifndef TRACE6
        #define TRACE6(p0, p1, p2, p3, p4, p5, p6)      TRACE(p0, p1, p2, p3, p4, p5, p6)
    #endif

#else

    #ifndef TRACE
        #define TRACE
    #endif

    #ifndef TRACE0
        #define TRACE0(p0)
    #endif

    #ifndef TRACE1
        #define TRACE1(p0, p1)
    #endif

    #ifndef TRACE2
        #define TRACE2(p0, p1, p2)
    #endif

    #ifndef TRACE3
        #define TRACE3(p0, p1, p2, p3)
    #endif

    #ifndef TRACE4
        #define TRACE4(p0, p1, p2, p3, p4)
    #endif

    #ifndef TRACE5
        #define TRACE5(p0, p1, p2, p3, p4, p5)
    #endif

    #ifndef TRACE6
        #define TRACE6(p0, p1, p2, p3, p4, p5, p6)
    #endif

#endif



// EOF
