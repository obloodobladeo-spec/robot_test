/* This application has no console/filesystem transport. Fail explicitly. */
#include <errno.h>
#include <sys/types.h>

int _close(int fd)
{
    (void)fd;
    errno = ENOSYS;
    return -1;
}

off_t _lseek(int fd, off_t offset, int whence)
{
    (void)fd;
    (void)offset;
    (void)whence;
    errno = ENOSYS;
    return (off_t)-1;
}

int _read(int fd, void *buffer, size_t count)
{
    (void)fd;
    (void)buffer;
    (void)count;
    errno = ENOSYS;
    return -1;
}

int _write(int fd, const void *buffer, size_t count)
{
    (void)fd;
    (void)buffer;
    (void)count;
    errno = ENOSYS;
    return -1;
}
