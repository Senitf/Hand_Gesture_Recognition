add_library(HandTrackingClient STATIC HandTrackingClient.h HandTrackingClient.cpp HandTrackingMessage.h HandTrackingMessage.cpp  HandTrackingListener.h Threads.h Threads.cpp VecMath.h VecMath.cpp OpenCVCamera.h OpenCVCamera.cpp) 
target_link_libraries(HandTrackingClient)
add_definitions(-DWINVER=0x0600 -D_WIN32_WINNT=0x0600 -D_CRT_SECURE_NO_WARNINGS)
