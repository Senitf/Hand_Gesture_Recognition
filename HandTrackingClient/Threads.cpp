#include "HandTrackingClient/Threads.h"
#include <cassert>

#ifndef _WIN32
#include <unistd.h>
#endif

namespace HandTrackingClient
{

#if defined(_WIN32)
// Mutex implementation
Mutex::Mutex()
{
    hMutex_ = CreateMutex( 
        NULL,              // default security attributes
        FALSE,             // initially not owned
        NULL);             // unnamed mutex
}

Mutex::~Mutex()
{
    CloseHandle (hMutex_);
}

// SimpleLock implementation
SimpleLock::SimpleLock(Mutex& mutex) 
    : mutex_(mutex)
{ 
    WaitForSingleObject (mutex_.hMutex_, INFINITE);
}

SimpleLock::~SimpleLock()
{
    ReleaseMutex (mutex_.hMutex_);
}

DWORD
Thread::wrapperFunction(LPVOID lpParam)
{
    ((Thread*) lpParam)->f_();
    return TRUE;
}

// Thread implementation:
Thread::Thread(std::function<void (void)> f) 
    : f_(f)
{
    hThread_ = CreateThread( 
                 NULL,       // default security attributes
                 0,          // default stack size
                 (LPTHREAD_START_ROUTINE) wrapperFunction, 
                 this,       // no thread function arguments
                 0,          // default creation flags
                 NULL);      // don't care about thread ID
}

Thread::~Thread()
{
    CloseHandle(hThread_);
}

void 
Thread::join() 
{
    WaitForSingleObject(hThread_, INFINITE);
}

void 
Thread::sleep(short millis) 
{ 
    Sleep(millis); 
}
#else // _WIN32
Mutex::Mutex()
    : posixMutex_ (PTHREAD_MUTEX_INITIALIZER)
{
}

Mutex::~Mutex()
{
}

// SimpleLock implementation
SimpleLock::SimpleLock (Mutex& mutex)
    : mutex_(mutex)
{
    int result = pthread_mutex_lock(&mutex_.posixMutex_);
    // Ideally, we'd do something useful with this error:
    assert (result == 0);
}

SimpleLock::~SimpleLock()
{
    int result = pthread_mutex_unlock(&mutex_.posixMutex_);
    assert (result == 0);
}

void*
Thread::wrapperFunction(void* lpParam)
{
    ((Thread*) lpParam)->f_();
    return nullptr;
}

// Thread implementation:
Thread::Thread(std::function<void (void)> f) 
    : f_(f)
{
    pthread_attr_t  attr;
    int returnVal = pthread_attr_init (&attr);
    assert (returnVal == 0);
    
    int threadError = pthread_create(&posixThreadId_, &attr, &wrapperFunction, this);
    assert (threadError == 0);
}

Thread::~Thread()
{
}

void 
Thread::join() 
{
    void* retVal;
    int result = pthread_join (posixThreadId_, &retVal);
    assert (result == 0);
}

void 
Thread::sleep(short millis) 
{
    usleep(static_cast<useconds_t>(millis)*static_cast<useconds_t>(1000));
}


#endif

} // namespace HandTrackingClient
