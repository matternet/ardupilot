/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  compatibility with posix APIs using AP_Filesystem
 */
#pragma once

#include <sys/types.h>

#if defined(CHIBIOS_BOARD_NAME)
#ifdef __cplusplus
extern "C" {
#endif

/*
  these are here to allow lua to build on HAL_ChibiOS
 */

typedef struct _file FILE;

int fprintf(FILE *stream, const char *format, ...);
int fflush(FILE *stream);
size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
size_t fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream);
int fputs(const char *s, FILE *stream);
char *fgets(char *s, int size, FILE *stream);
void clearerr(FILE *stream);
FILE *fopen(const char *pathname, const char *mode);
int fseek(FILE *stream, long offset, int whence);
int ferror(FILE *stream);
int fclose(FILE *stream);
FILE *tmpfile(void);
int sprintf(char *str, const char *format, ...);
int getc(FILE *stream);
int ungetc(int c, FILE *stream);

#define stdin ((FILE*)1)
#define stdout ((FILE*)2)
#define stderr ((FILE*)3)

#define BUFSIZ 256
#define EOF (-1)

#ifndef SEEK_SET
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2
#endif
    
#ifdef __cplusplus
}
#endif

#endif
