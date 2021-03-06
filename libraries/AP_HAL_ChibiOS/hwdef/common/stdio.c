/*
 * Copyright (C) Siddharth Bharat Purohit 2017
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  wrappers for stdio functions

  Relies on linker wrap options

  Note that not all functions that have been wrapped are implemented
  here. The others are wrapped to ensure the function is not used
  without an implementation. If we need them then we can implement as
  needed.
 */
#include <string.h>
#include <hal.h>
#include <memstreams.h>
#include <chprintf.h>
#include <ctype.h>
#include "stdio.h"

int vsnprintf(char *str, size_t size, const char *fmt, va_list ap)
{
  int retval = 0;
#ifndef HAL_NO_PRINTF
  MemoryStream ms;
  BaseSequentialStream *chp;
  size_t size_wo_nul;

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
#else
  (void)str;
  (void)size;
  (void)fmt;
  (void)ap;
#endif
  return retval;
}

int snprintf(char *str, size_t size, const char *fmt, ...)
{
#ifndef HAL_NO_PRINTF
   va_list arg;
   int done;
 
   va_start (arg, fmt);
   done =  vsnprintf(str, size, fmt, arg);
   va_end (arg);
 
   return done;
#else
   (void)str;
   (void)size;
   (void)fmt;
   return 0;
#endif
}

int vasprintf(char **strp, const char *fmt, va_list ap)
{
#ifndef HAL_NO_PRINTF
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
#else
    (void)strp;
    (void)fmt;
    (void)ap;
    return 0;
#endif
}

int asprintf(char **strp, const char *fmt, ...)
{
#ifndef HAL_NO_PRINTF
    va_list ap;
    va_start(ap, fmt);
    int ret = vasprintf(strp, fmt, ap);
    va_end(ap);
    return ret;
#else
    (void)strp;
    (void)fmt;
    return 0;
#endif
}

int vprintf(const char *fmt, va_list arg)
{
#ifdef HAL_STDOUT_SERIAL
  return chvprintf ((BaseSequentialStream*)&HAL_STDOUT_SERIAL, fmt, arg);
#else
  (void)arg;
  return strlen(fmt);
#endif
}

// hook to allow for printf() on systems without HAL_STDOUT_SERIAL
int (*vprintf_console_hook)(const char *fmt, va_list arg) = vprintf;

int printf(const char *fmt, ...)
{
#ifndef HAL_NO_PRINTF
   va_list arg;
   int done;
 
   va_start (arg, fmt);
   done =  vprintf_console_hook(fmt, arg);
   va_end (arg);
 
   return done;
#else
   (void)fmt;
   return 0;
#endif
}

//just a stub
int 
scanf (const char *fmt, ...)
{
    (void)fmt;
    return 0;
}
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

static char *
_getbase(char *p, int *basep)
{
  if (p[0] == '0') {
    switch (p[1]) {
    case 'x':
      *basep = 16;
      break;
    case 't': case 'n':
      *basep = 10;
      break;
    case 'o':
      *basep = 8;
      break;
    default:
      *basep = 10;
      return (p);
    }
    return (p + 2);
  }
  *basep = 10;
  return (p);
}

static int16_t
_atob (uint32_t *vp, char *p, int base)
{
  uint32_t value, v1, v2;
  char *q, tmp[20];
  int digit;

  if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
    base = 16;
    p += 2;
  }

  if (base == 16 && (q = strchr (p, '.')) != 0) {
      if ((unsigned)(q - p) > (unsigned)(sizeof(tmp) - 1))
      return (0);

    strncpy (tmp, p, q - p);
    tmp[q - p] = '\0';
    if (!_atob (&v1, tmp, 16))
      return (0);

    q++;
    if (strchr (q, '.'))
      return (0);

    if (!_atob (&v2, q, 16))
      return (0);
    *vp = (v1 << 16) + v2;
    return (1);
  }

  value = *vp = 0;
  for (; *p; p++) {
    if (*p >= '0' && *p <= '9')
      digit = *p - '0';
    else if (*p >= 'a' && *p <= 'f')
      digit = *p - 'a' + 10;
    else if (*p >= 'A' && *p <= 'F')
      digit = *p - 'A' + 10;
    else
      return (0);

    if (digit >= base)
      return (0);
    value *= base;
    value += digit;
  }
  *vp = value;
  return (1);
}

/*
 *  atob(vp,p,base) 
 *      converts p to binary result in vp, rtn 1 on success
 */
int16_t
atob(uint32_t *vp, char *p, int base)
{
  uint32_t  v;

  if (base == 0)
    p = _getbase (p, &base);
  if (_atob (&v, p, base)) {
    *vp = v;
    return (1);
  }
  return (0);
}


