/*
  ArduPilot filesystem interface for systems using the FATFS filesystem
 */
#include "AP_Filesystem.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

#if HAVE_FILESYSTEM_SUPPORT && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#if 0
#define debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

#define MAX_FILES 16
struct __file {
    char    *buf;       /* buffer pointer */
    unsigned char unget;    /* ungetc() buffer */
    uint8_t flags;      /* flags, see below */
#define __SRD   0x0001      /* OK to read */
#define __SWR   0x0002      /* OK to write */
#define __SSTR  0x0004      /* this is an sprintf/snprintf string */
#define __SPGM  0x0008      /* fmt string is in progmem */
#define __SERR  0x0010      /* found error */
#define __SEOF  0x0020      /* found EOF */
#define __SUNGET 0x040      /* ungetc() happened */
#define __SMALLOC 0x80      /* handle is malloc()ed */
    int size;       /* size of buffer */
    int len;        /* characters read or written so far */
    int (*put)(char, struct __file *);  				/* write one char to device */
    int (*get)(struct __file *);    					/* read one char from device */
    void    *udata;     /* User defined and accessible data. */
};

#define fdev_set_udata(stream, u) do { (stream)->udata = u; } while(0)
#define fdev_get_udata(stream) ((stream)->udata)

#define FATFS_R (S_IRUSR | S_IRGRP | S_IROTH)	/*< FatFs Read perms */
#define FATFS_W (S_IWUSR | S_IWGRP | S_IWOTH)	/*< FatFs Write perms */
#define FATFS_X (S_IXUSR | S_IXGRP | S_IXOTH)	/*< FatFs Execute perms */

///@brief  device status flags
#define _FDEV_EOF (-1)
#define _FDEV_ERR (-2)

//@brief device read/write flags
#define _FDEV_SETUP_READ  __SRD /**< fdev_setup_stream() with read intent */
#define _FDEV_SETUP_WRITE __SWR /**< fdev_setup_stream() with write intent */
#define _FDEV_SETUP_RW    (__SRD|__SWR) /**< fdev_setup_stream() with read/write intent */

typedef struct __file FILE;
FILE *__iob[MAX_FILES];

static bool isatty(int fileno)
{
    if (fileno >= 0 && fileno <= 2) {
        return true;
    }
    return false;
}

// =============================================
/// @brief Allocate a POSIX FILE descriptor.
/// NOT POSIX
///
/// @return fileno on success.
/// @return -1 on failure with errno set.
static int new_file_descriptor( void )
{
    int i;
    FILE *stream;
    FIL *fh;

    for (i=0; i<MAX_FILES; ++i) {
        if (isatty(i)) {
            continue;
        }
        if ( __iob[i] == NULL) {
            stream = (FILE *) calloc(sizeof(FILE),1);
            if (stream == NULL) {
                errno = ENOMEM;
                return -1;
            }
            fh = (FIL *) calloc(sizeof(FIL),1);
            if (fh == NULL) {
                free(stream);
                errno = ENOMEM;
                return -1;
            }

            __iob[i]  = stream;
            fdev_set_udata(stream, (void *) fh);
            return i;
        }
    }
    errno = ENFILE;
    return -1;
}

static FILE *fileno_to_stream(int fileno)
{
    FILE *stream;
    if (fileno < 0 || fileno >= MAX_FILES) {
        errno = EBADF;
        return nullptr;
    }

    stream = __iob[fileno];
    if (stream == NULL) {
        errno = EBADF;
        return nullptr;
    }
    return stream;
}

static int free_file_descriptor(int fileno)
{
    FILE *stream;
    FIL *fh;

    if (isatty( fileno )) {
        errno = EBADF;
        return -1;
    }

    // checks if fileno out of bounds
    stream = fileno_to_stream(fileno);
    if (stream == NULL) {
        return -1;
    }

    fh = (FIL *)fdev_get_udata(stream);

    if (fh != NULL) {
        free(fh);
    }

    if (stream->buf != NULL && stream->flags & __SMALLOC) {
        free(stream->buf);
    }

    __iob[fileno]  = NULL;
    free(stream);
    return fileno;
}

static FIL *fileno_to_fatfs(int fileno)
{
    FILE *stream;
    FIL *fh;

    if (isatty( fileno )) {
        errno = EBADF;
        return nullptr;
    }

    // checks if fileno out of bounds
    stream = fileno_to_stream(fileno);
    if ( stream == NULL ) {
        return nullptr;
    }

    fh = (FIL *)fdev_get_udata(stream);
    if (fh == NULL) {
        errno = EBADF;
        return nullptr;
    }
    return fh;
}

static int fatfs_to_errno( FRESULT Result )
{
    switch ( Result ) {
    case FR_OK:              /* FatFS (0) Succeeded */
        return 0;          /* POSIX OK */
    case FR_DISK_ERR:        /* FatFS (1) A hard error occurred in the low level disk I/O layer */
        return (EIO);        /* POSIX Input/output error (POSIX.1) */

    case FR_INT_ERR:         /* FatFS (2) Assertion failed */
        return (EPERM);      /* POSIX Operation not permitted (POSIX.1) */

    case FR_NOT_READY:       /* FatFS (3) The physical drive cannot work */
        return (EBUSY);      /* POSIX Device or resource busy (POSIX.1) */

    case FR_NO_FILE:         /* FatFS (4) Could not find the file */
        return (ENOENT);     /* POSIX No such file or directory (POSIX.1) */

    case FR_NO_PATH:         /* FatFS (5) Could not find the path */
        return (ENOENT);     /* POSIX No such file or directory (POSIX.1) */

    case FR_INVALID_NAME:    /* FatFS (6) The path name format is invalid */
        return (EINVAL);     /* POSIX Invalid argument (POSIX.1) */

    case FR_DENIED:          /* FatFS (7) Access denied due to prohibited access or directory full */
        return (EACCES);     /* POSIX Permission denied (POSIX.1) */

    case FR_EXIST:           /* file exists */
        return (EEXIST);     /* file exists */

    case FR_INVALID_OBJECT:  /* FatFS (9) The file/directory object is invalid */
        return (EINVAL);     /* POSIX Invalid argument (POSIX.1) */

    case FR_WRITE_PROTECTED: /* FatFS (10) The physical drive is write protected */
        return (EROFS);      /* POSIX Read-only filesystem (POSIX.1) */

    case FR_INVALID_DRIVE:   /* FatFS (11) The logical drive number is invalid */
        return (ENXIO);      /* POSIX No such device or address (POSIX.1) */

    case FR_NOT_ENABLED:     /* FatFS (12) The volume has no work area */
        return (ENOSPC);     /* POSIX No space left on device (POSIX.1) */

    case FR_NO_FILESYSTEM:   /* FatFS (13) There is no valid FAT volume */
        return (ENXIO);      /* POSIX No such device or address (POSIX.1) */

    case FR_MKFS_ABORTED:    /* FatFS (14) The f_mkfs() aborted due to any parameter error */
        return (EINVAL);     /* POSIX Invalid argument (POSIX.1) */

    case FR_TIMEOUT:         /* FatFS (15) Could not get a grant to access the volume within defined period */
        return (EBUSY);      /* POSIX Device or resource busy (POSIX.1) */

    case FR_LOCKED:          /* FatFS (16) The operation is rejected according to the file sharing policy */
        return (EBUSY);      /* POSIX Device or resource busy (POSIX.1) */


    case FR_NOT_ENOUGH_CORE: /* FatFS (17) LFN working buffer could not be allocated */
        return (ENOMEM);     /* POSIX Not enough space (POSIX.1) */

    case FR_TOO_MANY_OPEN_FILES:/* FatFS (18) Number of open files > _FS_SHARE */
        return (EMFILE);     /* POSIX Too many open files (POSIX.1) */

    case FR_INVALID_PARAMETER:/* FatFS (19) Given parameter is invalid */
        return (EINVAL);     /* POSIX Invalid argument (POSIX.1) */

    }
    return (EBADMSG);            /* POSIX Bad message (POSIX.1) */
}

static int fatfs_getc(FILE *stream)
{
    WITH_SEMAPHORE(AP::FS().get_sem());

    FIL *fh;
    UINT size;
    int res;
    uint8_t c;

    errno = 0;

    if (stream == NULL) {
        errno = EBADF;                            // Bad File Number
        return (EOF);
    }

    fh = (FIL *) fdev_get_udata(stream);
    if (fh == NULL) {
        errno = EBADF;                            // Bad File Number
        return (EOF);
    }

    res = f_read(fh, &c, 1, (UINT *) &size);
    if ( res != FR_OK || size != 1) {
        errno = fatfs_to_errno((FRESULT)res);
        stream->flags |= __SEOF;
        return (EOF);
    }
    return (c & 0xff);
}

static int fatfs_putc(char c, FILE *stream)
{
    WITH_SEMAPHORE(AP::FS().get_sem());

    int res;
    FIL *fh;
    UINT size;

    errno = 0;
    if (stream == NULL) {
        errno = EBADF;                            // Bad File Number
        return (EOF);
    }

    fh = (FIL *) fdev_get_udata(stream);
    if (fh == NULL) {
        errno = EBADF;                            // Bad File Number
        return (EOF);
    }

    res = f_write(fh, &c, 1, (UINT *)  &size);
    if ( res != FR_OK || size != 1) {
        errno = fatfs_to_errno((FRESULT)res);
        stream->flags |= __SEOF;
        return (EOF);
    }
    return c;
}

int AP_Filesystem::open(const char *pathname, int flags)
{
    int fileno;
    int fatfs_modes;
    FILE *stream;
    FIL *fh;
    int res;

    WITH_SEMAPHORE(sem);

    errno = 0;
    debug("Open %s 0x%x", pathname, flags);

    if ((flags & O_ACCMODE) == O_RDWR) {
        fatfs_modes = FA_READ | FA_WRITE;
    } else if ((flags & O_ACCMODE) == O_RDONLY) {
        fatfs_modes = FA_READ;
    } else {
        fatfs_modes = FA_WRITE;
    }

    if (flags & O_CREAT) {
        if (flags & O_TRUNC) {
            fatfs_modes |= FA_CREATE_ALWAYS;
        } else {
            fatfs_modes |= FA_OPEN_ALWAYS;
        }
    }

    fileno = new_file_descriptor();

    // checks if fileno out of bounds
    stream = fileno_to_stream(fileno);
    if (stream == NULL) {
        free_file_descriptor(fileno);
        return -1;
    }

    // fileno_to_fatfs checks for fileno out of bounds
    fh = fileno_to_fatfs(fileno);
    if (fh == NULL) {
        free_file_descriptor(fileno);
        errno = EBADF;
        return -1;
    }
    res = f_open(fh, pathname, (BYTE) (fatfs_modes & 0xff));
    if (res != FR_OK) {
        errno = fatfs_to_errno((FRESULT)res);
        free_file_descriptor(fileno);
        return -1;
    }
    if (flags & O_APPEND) {
        ///  Seek to end of the file
        res = f_lseek(fh, f_size(fh));
        if (res != FR_OK) {
            errno = fatfs_to_errno((FRESULT)res);
            f_close(fh);
            free_file_descriptor(fileno);
            return -1;
        }
    }

    if ((flags & O_ACCMODE) == O_RDWR) {
        // FIXME fdevopen should do this
        stream->put = fatfs_putc;
        stream->get = fatfs_getc;
        stream->flags = _FDEV_SETUP_RW;
    } else if ((flags & O_ACCMODE) == O_RDONLY) {
        // FIXME fdevopen should do this
        stream->put = NULL;
        stream->get = fatfs_getc;
        stream->flags = _FDEV_SETUP_READ;
    } else {
        // FIXME fdevopen should do this
        stream->put = fatfs_putc;
        stream->get = NULL;
        stream->flags = _FDEV_SETUP_WRITE;
    }

    debug("Open %s -> %d", pathname, fileno);

    return fileno;
}

int AP_Filesystem::close(int fileno)
{
    FILE *stream;
    FIL *fh;
    int res;

    WITH_SEMAPHORE(sem);

    errno = 0;

    // checks if fileno out of bounds
    stream = fileno_to_stream(fileno);
    if (stream == NULL) {
        return -1;
    }

    // fileno_to_fatfs checks for fileno out of bounds
    fh = fileno_to_fatfs(fileno);
    if (fh == NULL) {
        return -1;
    }
    res = f_close(fh);
    free_file_descriptor(fileno);
    if (res != FR_OK) {
        errno = fatfs_to_errno((FRESULT)res);
        return -1;
    }
    return 0;
}

ssize_t AP_Filesystem::read(int fd, void *buf, size_t count)
{
    UINT size;
    UINT bytes = count;
    int res;
    FIL *fh;

    WITH_SEMAPHORE(sem);

    if (count > 0) {
        *(char *) buf = 0;
    }

    errno = 0;

    // fileno_to_fatfs checks for fd out of bounds
    fh = fileno_to_fatfs(fd);
    if ( fh == NULL ) {
        errno = EBADF;
        return -1;
    }

    res = f_read(fh, (void *) buf, bytes, &size);
    if (res != FR_OK) {
        errno = fatfs_to_errno((FRESULT)res);
        return -1;
    }
    return (ssize_t)size;
}

ssize_t AP_Filesystem::write(int fd, const void *buf, size_t count)
{
    UINT size;
    UINT bytes = count;
    FRESULT res;
    FIL *fh;
    errno = 0;

    WITH_SEMAPHORE(sem);

    // fileno_to_fatfs checks for fd out of bounds
    fh = fileno_to_fatfs(fd);
    if ( fh == NULL ) {
        errno = EBADF;
        return -1;
    }

    res = f_write(fh, buf, bytes, &size);
    if (res != FR_OK) {
        errno = fatfs_to_errno(res);
        return -1;
    }
    return (ssize_t)size;
}

int AP_Filesystem::fsync(int fileno)
{
    FILE *stream;
    FIL *fh;
    int res;

    WITH_SEMAPHORE(sem);

    errno = 0;

    // checks if fileno out of bounds
    stream = fileno_to_stream(fileno);
    if (stream == NULL) {
        return -1;
    }

    // fileno_to_fatfs checks for fileno out of bounds
    fh = fileno_to_fatfs(fileno);
    if (fh == NULL) {
        return -1;
    }
    res = f_sync(fh);
    if (res != FR_OK) {
        errno = fatfs_to_errno((FRESULT)res);
        return -1;
    }
    return 0;
}

off_t AP_Filesystem::lseek(int fileno, off_t position, int whence)
{
    FRESULT res;
    FIL *fh;
    errno = 0;
    FILE *stream;

    WITH_SEMAPHORE(sem);

    // fileno_to_fatfs checks for fd out of bounds
    fh = fileno_to_fatfs(fileno);
    if (fh == NULL) {
        errno = EMFILE;
        return -1;
    }
    if (isatty(fileno)) {
        return -1;
    }


    stream = fileno_to_stream(fileno);
    stream->flags |= __SUNGET;

    if (whence == SEEK_END) {
        position += f_size(fh);
    } else if (whence==SEEK_CUR) {
        position += fh->fptr;
    }

    res = f_lseek(fh, position);
    if (res) {
        errno = fatfs_to_errno(res);
        return -1;
    }
    return fh->fptr;
}

/*
  mktime replacement from Samba
 */
static time_t replace_mktime(const struct tm *t)
{
    time_t  epoch = 0;
    int n;
    int mon [] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }, y, m, i;
    const unsigned MINUTE = 60;
    const unsigned HOUR = 60*MINUTE;
    const unsigned DAY = 24*HOUR;
    const unsigned YEAR = 365*DAY;

    if (t->tm_year < 70) {
        return (time_t)-1;
    }

    n = t->tm_year + 1900 - 1;
    epoch = (t->tm_year - 70) * YEAR +
            ((n / 4 - n / 100 + n / 400) - (1969 / 4 - 1969 / 100 + 1969 / 400)) * DAY;

    y = t->tm_year + 1900;
    m = 0;

    for (i = 0; i < t->tm_mon; i++) {
        epoch += mon [m] * DAY;
        if (m == 1 && y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) {
            epoch += DAY;
        }

        if (++m > 11) {
            m = 0;
            y++;
        }
    }

    epoch += (t->tm_mday - 1) * DAY;
    epoch += t->tm_hour * HOUR + t->tm_min * MINUTE + t->tm_sec;

    return epoch;
}

static time_t fat_time_to_unix(uint16_t date, uint16_t time)
{
    struct tm tp;
    time_t unix;

    memset(&tp, 0, sizeof(struct tm));

    tp.tm_sec = (time << 1) & 0x3e;               // 2 second resolution
    tp.tm_min = ((time >> 5) & 0x3f);
    tp.tm_hour = ((time >> 11) & 0x1f);
    tp.tm_mday = (date & 0x1f);
    tp.tm_mon = ((date >> 5) & 0x0f) - 1;
    tp.tm_year = ((date >> 9) & 0x7f) + 80;
    unix = replace_mktime( &tp );
    return unix;
}

int AP_Filesystem::stat(const char *name, struct stat *buf)
{
    FILINFO info;
    int res;
    time_t epoch;
    uint16_t mode;

    WITH_SEMAPHORE(sem);

    errno = 0;

    // f_stat does not handle / or . as root directory
    if (strcmp(name,"/") == 0 || strcmp(name,".") == 0) {
        buf->st_atime = 0;
        buf->st_mtime = 0;
        buf->st_ctime = 0;
        buf->st_uid= 0;
        buf->st_gid= 0;
        buf->st_size = 0;
        buf->st_mode = S_IFDIR;
        return 0;
    }

    res = f_stat(name, &info);
    if (res != FR_OK) {
        errno = fatfs_to_errno((FRESULT)res);
        return -1;
    }

    buf->st_size = info.fsize;
    epoch = fat_time_to_unix(info.fdate, info.ftime);
    buf->st_atime = epoch;                        // Access time
    buf->st_mtime = epoch;                        // Modification time
    buf->st_ctime = epoch;                        // Creation time

    // We only handle read only case
    mode = (FATFS_R | FATFS_X);
    if ( !(info.fattrib & AM_RDO)) {
        mode |= (FATFS_W);    // enable write if NOT read only
    }

    if (info.fattrib & AM_SYS) {
        buf->st_uid= 0;
        buf->st_gid= 0;
    }
    {
        buf->st_uid=1000;
        buf->st_gid=1000;
    }

    if (info.fattrib & AM_DIR) {
        mode |= S_IFDIR;
    } else {
        mode |= S_IFREG;
    }
    buf->st_mode = mode;

    return 0;
}

int AP_Filesystem::unlink(const char *pathname)
{
    WITH_SEMAPHORE(sem);

    errno = 0;
    int res = f_unlink(pathname);
    if (res != FR_OK) {
        errno = fatfs_to_errno((FRESULT)res);
        return -1;
    }
    return 0;
}

int AP_Filesystem::mkdir(const char *pathname)
{
    WITH_SEMAPHORE(sem);

    errno = 0;

    int res = f_mkdir(pathname);
    if (res != FR_OK) {
        errno = fatfs_to_errno((FRESULT)res);
        return -1;
    }

    return 0;
}

/*
  wrapper structure to associate a dirent with a DIR
 */
struct DIR_Wrapper {
    DIR d; // must be first structure element
    struct dirent de;
};

DIR *AP_Filesystem::opendir(const char *pathdir)
{
    WITH_SEMAPHORE(sem);

    debug("Opendir %s", pathdir);
    struct DIR_Wrapper *ret = new DIR_Wrapper;
    if (!ret) {
        return nullptr;
    }
    int res = f_opendir(&ret->d, pathdir);
    if (res != FR_OK) {
        errno = fatfs_to_errno((FRESULT)res);
        delete ret;
        return nullptr;
    }
    debug("Opendir %s -> %p", pathdir, ret);
    return &ret->d;
}

struct dirent *AP_Filesystem::readdir(DIR *dirp)
{
    WITH_SEMAPHORE(sem);

    struct DIR_Wrapper *d = (struct DIR_Wrapper *)dirp;
    if (!d) {
        errno = EINVAL;
        return nullptr;
    }
    FILINFO fno;
    int len;
    int res;

    d->de.d_name[0] = 0;
    res = f_readdir ( dirp, &fno );
    if (res != FR_OK || fno.fname[0] == 0) {
        errno = fatfs_to_errno((FRESULT)res);
        return nullptr;
    }
    len = strlen(fno.fname);
    strncpy(d->de.d_name,fno.fname,len);
    d->de.d_name[len] = 0;
    debug("readdir -> %s", d->de.d_name);
    return &d->de;
}

int AP_Filesystem::closedir(DIR *dirp)
{
    WITH_SEMAPHORE(sem);

    struct DIR_Wrapper *d = (struct DIR_Wrapper *)dirp;
    if (!d) {
        errno = EINVAL;
        return -1;
    }
    int res = f_closedir (dirp);
    delete d;
    if (res != FR_OK) {
        errno = fatfs_to_errno((FRESULT)res);
        return -1;
    }
    debug("closedir");
    return 0;
}

// return free disk space in bytes
int64_t AP_Filesystem::disk_free(const char *path)
{
    WITH_SEMAPHORE(sem);

    FATFS *fs;
    DWORD fre_clust, fre_sect;


    /* Get volume information and free clusters of drive 1 */
    FRESULT res = f_getfree("/", &fre_clust, &fs);
    if (res) {
        return res;
    }

    /* Get total sectors and free sectors */
    fre_sect = fre_clust * fs->csize;
    return (int64_t)(fre_sect)*512;
}

// return total disk space in bytes
int64_t AP_Filesystem::disk_space(const char *path)
{
    WITH_SEMAPHORE(sem);

    FATFS *fs;
    DWORD fre_clust, tot_sect;

    /* Get volume information and free clusters of drive 1 */
    FRESULT res = f_getfree("/", &fre_clust, &fs);
    if (res) {
        return (res);
    }

    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    return (int64_t)(tot_sect)*512;
}

static int fileno(FILE *stream)
{
    WITH_SEMAPHORE(AP::FS().get_sem());

    int fileno;

    if (stream == NULL) {
        errno = EBADF;
        return -1;
    }

    for (fileno=0; fileno<MAX_FILES; ++fileno) {
        if ( __iob[fileno] == stream) {
            return fileno;
        }
    }
    return -1;
}

/*
  wrappers to replace libc functions
 */
extern "C" {
    size_t __wrap_fread ( void *ptr, size_t size, size_t nmemb, FILE *stream );
    int __wrap_fclose(FILE *stream);
    FILE * __wrap_freopen ( const char * filename, const char * mode, FILE * stream );
    FILE *fopen(const char *path, const char *mode);
    int __wrap_fprintf(FILE *fp, const char *fmt, ...);
    int fputc(int c, FILE *stream);
    char *__wrap_strerror_r(int errnum, char *buf, size_t buflen);
}

size_t __wrap_fread(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
    size_t count = size * nmemb;
    int fn = fileno(stream);
    ssize_t ret;

    // read() checks for fn out of bounds
    ret = AP::FS().read(fn, ptr, count);
    if (ret < 0) {
        return 0;
    }

    return (size_t)ret;
}

int __wrap_fclose(FILE *stream)
{
    int fn = fileno(stream);
    if (fn < 0) {
        return EOF;
    }

    return AP::FS().close(fn);
}

#define modecmp(str, pat) (strcmp(str, pat) == 0 ? 1: 0)

static int posix_fopen_modes_to_open(const char *mode)
{
    int flag = 0;

    if (modecmp(mode,"r") || modecmp(mode,"rb")) {
        flag = O_RDONLY;
        return flag;
    }
    if (modecmp(mode,"r+") || modecmp(mode, "r+b" ) || modecmp(mode, "rb+" )) {
        flag = O_RDWR | O_TRUNC;
        return flag;
    }
    if (modecmp(mode,"w") || modecmp(mode,"wb")) {
        flag = O_WRONLY | O_CREAT | O_TRUNC;
        return flag;
    }
    if (modecmp(mode,"w+") || modecmp(mode, "w+b" ) || modecmp(mode, "wb+" )) {
        flag = O_RDWR | O_CREAT | O_TRUNC;
        return flag;
    }
    if (modecmp(mode,"a") || modecmp(mode,"ab")) {
        flag = O_WRONLY | O_CREAT | O_APPEND;
        return flag;
    }
    if (modecmp(mode,"a+") || modecmp(mode, "a+b" ) || modecmp(mode, "ab+" )) {
        flag = O_RDWR | O_CREAT | O_APPEND;
        return -1;
    }
    return -1;                                  // nvalid mode
}

FILE *fopen(const char *path, const char *mode)
{
    int flags = posix_fopen_modes_to_open(mode);
    int fileno = AP::FS().open(path, flags);

    // checks if fileno out of bounds
    return fileno_to_stream(fileno);
}

int fputc(int c, FILE *stream)
{
    WITH_SEMAPHORE(AP::FS().get_sem());
    return fatfs_putc(c,stream);
}

FILE * __wrap_freopen( const char * filename, const char * mode, FILE * stream )
{
    int fn = fileno(stream);
    int ret = AP::FS().close(fn);
    if (ret < 0) {
        return NULL;
    }
    return fopen(filename, mode);
}

int __wrap_fprintf(FILE *fp, const char *fmt, ...)
{
    va_list va;
    char* buf = NULL;
    int16_t len, i;
    va_start(va, fmt);
    len = vasprintf(&buf, fmt, va);
    if (len > 0) {
        for (i = 0; i < len; i++) {
            fputc(buf[i], fp);
        }
        free(buf);
    } else {
        va_end(va);
        return -1;
    }
    va_end(va);

    return len;
}

/// @brief POSIX strerror() -  convert POSIX errno to text with user message.
///
/// - man page strerror (3).
///
/// @param[in] errnum: error provided from <errno.h>
///
/// @return  char *

char *strerror(int errnum)
{
#define SWITCH_ERROR(errno) case errno: return const_cast<char *>(#errno); break
    switch (errnum) {
        SWITCH_ERROR(EPERM);
        SWITCH_ERROR(ENOENT);
        SWITCH_ERROR(ESRCH);
        SWITCH_ERROR(EINTR);
        SWITCH_ERROR(EIO);
        SWITCH_ERROR(ENXIO);
        SWITCH_ERROR(E2BIG);
        SWITCH_ERROR(ENOEXEC);
        SWITCH_ERROR(EBADF);
        SWITCH_ERROR(ECHILD);
        SWITCH_ERROR(EAGAIN);
        SWITCH_ERROR(ENOMEM);
        SWITCH_ERROR(EACCES);
        SWITCH_ERROR(EFAULT);
#ifdef ENOTBLK
        SWITCH_ERROR(ENOTBLK);
#endif // ENOTBLK
        SWITCH_ERROR(EBUSY);
        SWITCH_ERROR(EEXIST);
        SWITCH_ERROR(EXDEV);
        SWITCH_ERROR(ENODEV);
        SWITCH_ERROR(ENOTDIR);
        SWITCH_ERROR(EISDIR);
        SWITCH_ERROR(EINVAL);
        SWITCH_ERROR(ENFILE);
        SWITCH_ERROR(EMFILE);
        SWITCH_ERROR(ENOTTY);
        SWITCH_ERROR(ETXTBSY);
        SWITCH_ERROR(EFBIG);
        SWITCH_ERROR(ENOSPC);
        SWITCH_ERROR(ESPIPE);
        SWITCH_ERROR(EROFS);
        SWITCH_ERROR(EMLINK);
        SWITCH_ERROR(EPIPE);
        SWITCH_ERROR(EDOM);
        SWITCH_ERROR(ERANGE);
        SWITCH_ERROR(EBADMSG);
    }

#undef SWITCH_ERROR

    return NULL;
}

/// @brief POSIX strerror_r() -  convert POSIX errno to text with user message.
///
/// - man page strerror (3).
///
/// @param[in] errnum: index for sys_errlist[]
/// @param[in] buf: user buffer for error message
/// @param[in] buflen: length of user buffer for error message
///
/// @see sys_errlist[].
/// @return  char *

char *__wrap_strerror_r(int errnum, char *buf, size_t buflen)
{
        strncpy(buf, strerror(errnum), buflen);
        return(buf);
}


#endif // CONFIG_HAL_BOARD
