#ifndef SRC_DRIVERS_INFINIBAND_MLX_UTILS_INCLUDE_PUBLIC_MLX_DEBUG_H_
#define SRC_DRIVERS_INFINIBAND_MLX_UTILS_INCLUDE_PUBLIC_MLX_DEBUG_H_

#ifndef ASSEMBLY

extern void MlxDebugInitializeSerial();
extern void MlxDebugBreakSerial();
extern void _MlxDebugWaitInfinite(void);

#define MlxDebugBreak()	MlxDebugBreakSerial()

#define MlxDebugBreakCond(cond)	\
	if ( cond )\
	{\
		MlxDebugBreakSerial()\
	}

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

#endif

#endif /* SRC_DRIVERS_INFINIBAND_MLX_UTILS_INCLUDE_PUBLIC_MLX_DEBUG_H_ */
