# Add OpenGL dependency
include_directories(${OPENGL_INCLUDE_DIRS})

add_executable(DrawSkin DrawSkin.cpp)
 
target_link_libraries(DrawSkin ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES} HandTrackingClient ${COREFOUNDATION_LIBRARIES} ${IOKIT_LIBRARIES} ${COCOA_LIBRARIES})

