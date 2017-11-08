#include "posix.h"
#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

int vsnprintf(char *str, size_t size, const char *fmt, va_list ap);
int snprintf(char *str, size_t size, const char *fmt, ...);
int vasprintf(char **strp, const char *fmt, va_list ap);
int asprintf(char **strp, const char *fmt, ...);
int vprintf(const char *fmt, va_list arg);
int printf(const char *fmt, ...);


int sscanf (const char *buf, const char *fmt, ...);
int vsscanf (const char *buf, const char *s, va_list ap);
void *malloc(size_t size);
void *calloc(size_t nmemb, size_t size);
void free(void *ptr);
#ifdef __cplusplus
}
#endif