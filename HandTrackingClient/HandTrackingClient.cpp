#include "HandTrackingClient/HandTrackingClient.h"
#include "HandTrackingClient/Threads.h"

#include <iostream>
#include <sstream>
#include <cassert>
#include <cstdlib>
#include <cstring>

#ifdef WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif

#  include <winsock2.h>
#  include <Ws2tcpip.h>
#  include <stdio.h>

// Link with ws2_32.lib
#  pragma comment(lib, "Ws2_32.lib")
#else // Unix-like operating systems
#  include <unistd.h>
#  include <sys/socket.h>
#  include <sys/types.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#endif

namespace HandTrackingClient
{

#ifndef WIN32
const int INVALID_SOCKET = -1;
const int SOCKET_ERROR = -1;
#endif

class ClientImpl 
{
public:
    ClientImpl() : _stopClientThread(false), _socket(INVALID_SOCKET) { }
    ~ClientImpl();

    std::pair<bool, std::string> connect(const char* ipAddr, const uint16_t port = 1988);
    void addHandTrackingListener(HandTrackingListener* listener);
    void stop();

    void setHeadTransform (const Vector3f& trans, const Quaternionf& rot);

private:
    // Runs in a new thread
    void listen();

    std::vector<HandTrackingListener*> _listeners;

    std::unique_ptr<Thread> _clientThread;
    bool _stopClientThread;

#ifdef WIN32
    SOCKET _socket;
#else
    int _socket;
#endif
};

Client::Client() { _clientImpl.reset(new ClientImpl); }

Client::~Client() { }

std::pair<bool, std::string> Client::connect(const char* ipAddr, const uint16_t port)
{
    return _clientImpl->connect(ipAddr, port);
}

void Client::addHandTrackingListener(HandTrackingListener* listener)
{
    _clientImpl->addHandTrackingListener(listener);
}

void Client::stop()
{
    _clientImpl->stop();
}

void Client::setHeadTransform (const Vector3f& trans, const Quaternionf& rot)
{
    _clientImpl->setHeadTransform (trans, rot);
}

ClientImpl::~ClientImpl()
{
    stop();

#ifdef WIN32
    WSACleanup();
#endif
}

void
ClientImpl::setHeadTransform (const Vector3f& trans, const Quaternionf& rot)
{
    const size_t DEFAULT_BUFLEN = 512;
    char sendbuf[DEFAULT_BUFLEN];
    const int sendbuflen = DEFAULT_BUFLEN;

#ifdef WIN32
    int len = _snprintf_s (sendbuf, sendbuflen, _TRUNCATE, "HEAD_TRANSFORM   %f %f %f   %f %f %f %f\n",
                           trans.x, trans.y, trans.z,
                           rot.v.x, rot.v.y, rot.v.z, rot.w);
#else
    int len = snprintf (sendbuf, sendbuflen, "HEAD_TRANSFORM   %f %f %f   %f %f %f %f\n",
                        trans.x, trans.y, trans.z,
                        rot.v.x, rot.v.y, rot.v.z, rot.w);
#endif

    // In an ideal world this would probably be non-blocking.
    send (_socket, sendbuf, len, 0);
}

std::string
lastSocketError ()
{
#ifdef WIN32
    struct MsgBuf
    {
        MsgBuf() : lpMsgBuf(nullptr) {}
        ~MsgBuf() { LocalFree(lpMsgBuf); }
        CHAR* lpMsgBuf;
    } buf;

    const int errCode = WSAGetLastError();

    // Always do this in ASCII for now:
    FormatMessageA  (FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                     NULL,
                     errCode,
                     MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                     (LPSTR) &buf.lpMsgBuf,
                     0, 
                     NULL);
    if (buf.lpMsgBuf == nullptr)
    {
        std::ostringstream oss;
        oss << "Unknown error: " << errCode;
        return oss.str();
    }
    else
    {
        const std::string result (buf.lpMsgBuf);
        return result;
    }
#else
    return std::string (strerror (errno));
#endif
}

std::pair<bool, std::string> 
ClientImpl::connect(const char* ipAddr, const uint16_t port)
{
    int iResult;

    _socket = INVALID_SOCKET;

#ifdef WIN32
    WSADATA wsaData;

    //----------------------
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != NO_ERROR) 
        return make_pair(false, std::string("WSAStartup() failed: ") + lastSocketError());
#endif

    //----------------------
    // Create a SOCKET for connecting to server
    _socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (_socket == INVALID_SOCKET) 
        return make_pair(false, std::string("Error at socket(): ") + lastSocketError());

    //----------------------
    // The sockaddr_in structure specifies the address family,
    // IP address, and port of the server to be connected to.
    struct sockaddr_in clientService; 
    clientService.sin_family = AF_INET;
    clientService.sin_addr.s_addr = inet_addr( ipAddr );
    clientService.sin_port = htons( port );

    //----------------------
    // Connect to server.
#ifdef WIN32
    iResult = ::connect( _socket, (SOCKADDR*) &clientService, sizeof(clientService) );
#else
    iResult = ::connect( _socket, (struct sockaddr*) &clientService, sizeof(clientService) );
#endif

    if ( iResult == SOCKET_ERROR) 
    {
        const std::string errStr = lastSocketError();

#ifdef WIN32
        closesocket (_socket);
#else
        close (_socket);
#endif
        _socket = -1;
        return make_pair(false, std::string("Error in connect(): ") + errStr);
    }

    // Function object that calls the io_service::run method:
    class ListenFunctor
    {
    public:
        ListenFunctor (ClientImpl& impl) : _clientImpl(impl) {}
        void operator()() { _clientImpl.listen(); }

    private:
        ClientImpl& _clientImpl;
    };

    // Start a new thread to connect to the hand tracking server
    _clientThread.reset(
        new Thread(ListenFunctor(*this)));

    return std::make_pair(true, std::string());
}

void ClientImpl::stop()
{
    _stopClientThread = true;
    if (_clientThread.get() != nullptr) 
    {
        _clientThread->join();
        _clientThread.reset();
    }

    if (_socket != -1)
    {
#ifdef WIN32
        closesocket (_socket);
#else
        close (_socket);
#endif
        _socket = -1;
    }
}

void ClientImpl::addHandTrackingListener(HandTrackingListener* listener)
{
    _listeners.push_back (listener);
}

void ClientImpl::listen()
{
    const size_t DEFAULT_BUFLEN = 512;
    char recvbuf[DEFAULT_BUFLEN];
    const int recvbuflen = DEFAULT_BUFLEN;

    std::string remaining;

    do 
    {
        const int iResult = recv(_socket, recvbuf, recvbuflen, 0);
        if (iResult == 0)
        {
            std::cerr << "Connection closed" << std::endl;
            break;
        }
        else if (iResult < 0)
        {
            std::cerr << "recv() failed: " << lastSocketError() << std::endl;
            break;
        }
        assert (iResult <= recvbuflen);

        // The way sockets work, we have to be able to receive an arbitrary number
        // of characters in each recv() call, and then parse out the actual lines
        // of text.  
        const char* lineStart = recvbuf;
        const char* lineEnd = nullptr;
        const char* bufEnd = recvbuf + iResult;
        while ((lineEnd = (const char*) memchr(lineStart, '\n', bufEnd - lineStart)) != nullptr)
        {
            size_t length = lineEnd - lineStart;
            std::string resultString;
            resultString.reserve (remaining.size() + length);
            resultString.append (remaining);
            resultString.append (lineStart, length);
            remaining.clear();

            // Skip over the '\n'
            lineStart = lineEnd + 1;

            try
            {
                // Parse received messages from the hand tracking server 
                std::auto_ptr<HandTrackingMessage> msg (
                    HandTrackingMessage::deserialize(resultString));

                // Ignore messages we don't understand
                if (msg.get() == nullptr) 
                    continue;

                for (std::vector<HandTrackingListener*>::const_iterator listenerItr = _listeners.begin();
                     listenerItr != _listeners.end();
                     ++listenerItr)
                {
                    (*listenerItr)->handleEvent(*msg);
                }
            }
            catch (std::exception& e)
            {
                std::cerr << "Error parsing message: " << e.what() << "\n";
            }
        }

        if (lineStart != bufEnd)
            remaining.append (lineStart, bufEnd - lineStart);

    } while (!_stopClientThread);

    for (std::vector<HandTrackingListener*>::const_iterator listenerItr = _listeners.begin();
         listenerItr != _listeners.end();
         ++listenerItr)
        (*listenerItr)->handleConnectionClosed();
}

} // namespace HandTrackingAppClient

