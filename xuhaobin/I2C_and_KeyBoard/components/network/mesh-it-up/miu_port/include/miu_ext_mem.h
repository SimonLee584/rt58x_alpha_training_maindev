#ifndef APP_EXT_MEM_H
#define APP_EXT_MEM_H

#include <stddef.h> // for size_t

#ifdef __cplusplus
extern "C" {
#endif

void heapLockInit(void);
void* extMalloc(size_t size, const char* file, int line);
void extFree(void* ptr, const char* file, int line);
void extMemory();

#define xMalloc(size) extMalloc(size, __FILE__, __LINE__)
#define xFree(ptr)    extFree(ptr, __FILE__, __LINE__)

#ifdef __cplusplus
}
#endif

#endif // APP_EXT_MEM_H