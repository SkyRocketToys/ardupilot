/*
  wrappers for stdio functions

  Relies on linker wrap options

  Note that not all functions that have been wrapped are implemented
  here. The others are wrapped to ensure the function is not used
  without an implementation. If we need them then we can implement as
  needed.
 */
#include <posix.h>
#include <string.h>
#include <hal.h>
#include <memstreams.h>
#include <chprintf.h>
#include <ctype.h>

int vsnprintf(char *str, size_t size, const char *fmt, va_list ap)
{
  MemoryStream ms;
  BaseSequentialStream *chp;
  size_t size_wo_nul;
  int retval;

  if (size > 0)
    size_wo_nul = size - 1;
  else
    size_wo_nul = 0;

  /* Memory stream object to be used as a string writer, reserving one
     byte for the final zero.*/
  msObjectInit(&ms, (uint8_t *)str, size_wo_nul, 0);

  /* Performing the print operation using the common code.*/
  chp = (BaseSequentialStream *)(void *)&ms;

  retval = chvprintf(chp, fmt, ap);


  /* Terminate with a zero, unless size==0.*/
  if (ms.eos < size)
      str[ms.eos] = 0;

  /* Return number of bytes that would have been written.*/
  return retval;
}

int snprintf(char *str, size_t size, const char *fmt, ...)
{
   va_list arg;
   int done;
 
   va_start (arg, fmt);
   done =  vsnprintf(str, size, fmt, arg);
   va_end (arg);
 
   return done;
}

int vasprintf(char **strp, const char *fmt, va_list ap)
{
    int len = vsnprintf(NULL, 0, fmt, ap);
    if (len <= 0) {
        return -1;
    }
    char *buf = calloc(len+1, 1);
    if (!buf) {
        return -1;
    }
    vsnprintf(buf, len+1, fmt, ap);
    (*strp) = buf;
    return len;
}

int asprintf(char **strp, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int ret = vasprintf(strp, fmt, ap);
    va_end(ap);
    return ret;
}

int vprintf(const char *fmt, va_list arg)
{
  return chvprintf ((BaseSequentialStream*)&HAL_STDOUT_SERIAL, fmt, arg);
}

int printf(const char *fmt, ...)
{
   va_list arg;
   int done;
 
   va_start (arg, fmt);
   done =  vprintf(fmt, arg);
   va_end (arg);
 
   return done;
}

#define MAXLN 128
#define ISSPACE " \t\n\r\f\v"

/*
 *  sscanf(buf,fmt,va_alist)
 */
int 
sscanf (const char *buf, const char *fmt, ...)
{
    int             count;
    va_list ap;
    
    va_start (ap, fmt);
    count = vsscanf (buf, fmt, ap);
    va_end (ap);
    return (count);
}

/*
 *  vsscanf(buf,fmt,ap)
 */
int
vsscanf (const char *buf, const char *s, va_list ap)
{
    int             count, noassign, width, base, lflag;
    const char     *tc;
    char           *t, tmp[MAXLN];

    count = noassign = width = lflag = 0;
    while (*s && *buf) {
  while (isspace ((unsigned char)(*s)))
      s++;
  if (*s == '%') {
      s++;
      for (; *s; s++) {
    if (strchr ("dibouxcsefg%", *s))
        break;
    if (*s == '*')
        noassign = 1;
    else if (*s == 'l' || *s == 'L')
        lflag = 1;
    else if (*s >= '1' && *s <= '9') {
        for (tc = s; isdigit (*s); s++);
        strncpy (tmp, tc, s - tc);
        tmp[s - tc] = '\0';
        atob (&width, tmp, 10);
        s--;
    }
      }
      if (*s == 's') {
    while (isspace ((unsigned char)(*buf)))
        buf++;
    if (!width)
        width = strcspn (buf, ISSPACE);
    if (!noassign) {
        strncpy (t = va_arg (ap, char *), buf, width);
        t[width] = '\0';
    }
    buf += width;
      } else if (*s == 'c') {
    if (!width)
        width = 1;
    if (!noassign) {
        strncpy (t = va_arg (ap, char *), buf, width);
        t[width] = '\0';
    }
    buf += width;
      } else if (strchr ("dobxu", *s)) {
    while (isspace ((unsigned char)(*buf)))
        buf++;
    if (*s == 'd' || *s == 'u')
        base = 10;
    else if (*s == 'x')
        base = 16;
    else if (*s == 'o')
        base = 8;
    else if (*s == 'b')
        base = 2;
    if (!width) {
        if (isspace ((unsigned char)(*(s + 1))) || *(s + 1) == 0)
      width = strcspn (buf, ISSPACE);
        else
      width = strchr (buf, *(s + 1)) - buf;
    }
    strncpy (tmp, buf, width);
    tmp[width] = '\0';
    buf += width;
    if (!noassign)
        atob (va_arg (ap, uint32_t *), tmp, base);
      }
      if (!noassign)
    count++;
      width = noassign = lflag = 0;
      s++;
  } else {
      while (isspace ((unsigned char)(*buf)))
    buf++;
      if (*s != *buf)
    break;
      else
    s++, buf++;
  }
    }
    return (count);
}
