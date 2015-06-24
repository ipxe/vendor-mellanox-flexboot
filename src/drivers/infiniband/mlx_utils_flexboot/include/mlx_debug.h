/*
 * mlx_debug.h
 *
 *  Created on: Dec 16, 2014
 *      Author: moshikos
 */

#ifndef SRC_MLX_COMMON_INCLUDE_MLX_DEBUG_H_
#define SRC_MLX_COMMON_INCLUDE_MLX_DEBUG_H_

#ifndef ASSEMBLY
#ifdef MLX_DEBUG

/*
 * Extern declerations
 */

extern void MlxDebugInitializeSerial();
extern void MlxDebugBreakSerial();
extern void _MlxDebugWaitInfinite(void);

/*
 * Active Debug Routines
 */

#define MlxDebugBreak()	MlxDebugBreakSerial()

#define MlxDebugBreakOnce()	\
	{\
		static int MlxDebugBreakOnceFlag=0;\
		if ( MlxDebugBreakOnceFlag == 0 ) \
		{\
			MlxDebugBreakSerial();\
			MlxDebugBreakOnceFlag=1;\
		}\
	}

#define MlxDebugInitialize()	MlxDebugInitializeSerial()
#define MlxDebugWaitInfinite()	_MlxDebugWaitInfinite()

#else

#define MlxDebugBreak()
#define MlxDebugInitialize()
#define MlxDebugWaitInfinite()

#endif /* MLX_DEBUG */

#ifdef MLX_TRACE

/*
 * Debug Tracing Macros
 */

#define MlxTraceEnter() \
	printf("%s:%d - Enter\n",__FUNCTION__,__LINE__)

#define MlxTraceLeave() \
	printf("%s:%d - Leave\n",__FUNCTION__,__LINE__)

#define MlxTrace() \
	printf("%s:%d\n",__FUNCTION__,__LINE__)

#else

#define MlxTraceEnter()
#define MlxTraceLeave()
#define MlxTrace()

#endif /* MLX_TRACE */

#endif /* ASSEMBLY */
#endif /* SRC_MLX_COMMON_INCLUDE_MLX_DEBUG_H_ */
