#ifndef tool_h
#define tool_h
#include <stdint.h>
 // 0 = Ignore M6; 1 = Manual tool change; 2 = Manual tool change + TLS
#define TOOL_CHANGE_IGNORE       0
#define TOOL_CHANGE_MANUAL       1
#define TOOL_CHANGE_MANUAL_TLS   2
//
#define TOOL_IS_VALID            1
#define TOOL_NO_VALID            0
//
void toolInit();
void toolCurrentChange();
void toolProbeTLS();

#endif
