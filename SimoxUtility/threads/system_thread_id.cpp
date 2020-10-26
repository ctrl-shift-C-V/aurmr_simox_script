#include <syscall.h>
#include <unistd.h>

#include "system_thread_id.h"

namespace simox
{
    long system_thread_id()
    {
        return syscall(SYS_gettid);
    }
}

