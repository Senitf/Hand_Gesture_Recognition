#pragma once

#include "HandTrackingClient/HandTrackingListener.h"
#include "HandTrackingClient/HandTrackingMessage.h"

#include <string>
#include <memory>
#include <cstdint>

/**
\mainpage

\section Introduction

The hand-tracking server uses a simple network protocol to communicate with
your applications over a socket.  This API communicates with the server
and wraps the results out using C++.

\section Overview

The library operates using callbacks.  To get started, you'll need to 
derive from HandTrackingListener.  You can then create a 
HandTrackingClient which will connect to the server start up its own thread
which waits and passes messages (PinchMessage, PointMessage, etc.) back to
your HandTrackingListener via the callback interface.  

Each HandTrackingMessage contains the position and orientation of both hands. 
The coordinate space of the hand position is specified by the camera setup
stage.  For a standard camera setup, the x-axis points right; the y-axis points
up; and the z-axis points away from the screen. Units are in millimeters and
the origin is at the center of the checkerboard you used during calibration.
*/


namespace HandTrackingClient
{

// In case you want to use this as a standalone library, we hide the implementation
// in the CPP file so that you don't have to depend on asio or boost
// We use the private implementation idiom: http://c2.com/cgi/wiki?PimplIdiom
class ClientImpl;

class Client
{
public:
    Client();
    ~Client();

    //! \brief Attempts to connect to the hand tracking server.
    //!
    //! Starts a new thread to listen for hand tracking events if connection succeed.
    //!
    //! \returns false if connection fails.
    std::pair<bool, std::string> connect(const char *ipAddr = "127.0.0.1", const uint16_t port = 1988);

    //! \brief Registers the given listener with this client.
    void addHandTrackingListener(HandTrackingListener* listener);

    //! Stop listening for events
    void stop();

    void setHeadTransform (const Vector3f& trans, const Quaternionf& rot);

private:
    std::unique_ptr<ClientImpl> _clientImpl;
};

} // namespace HandTrackingClient

