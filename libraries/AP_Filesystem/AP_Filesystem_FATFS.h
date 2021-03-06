/*
  FATFS backend for AP_Filesystem
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stddef.h>
#include <ff.h>

// Seek offset macros
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

#define MAX_NAME_LEN _MAX_LFN

struct dirent {
   char           d_name[MAX_NAME_LEN]; /* filename */
};
