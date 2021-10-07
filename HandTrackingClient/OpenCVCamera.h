#pragma once

#include "HandTrackingClient/VecMath.h"

namespace HandTrackingClient
{

//! \brief A class representing the intrinsic/extrinsic parameters of the depth camera.
//! 
//! Our camera model is identical to the one used in OpenCV
//! (and in OpenNI).  A full description can be found here:
//! http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
//!
//! The extrinsic matrix is set up such that world space corresponds
//! to the desk frame used in hand tracking, with y pointing up and 
//! z pointing toward the user.  
//!
//! Everything here will be in double precision, because this is
//! necessary for dealing robustly with distortion (some distortion
//! terms use the 6th power of the radius, which would cause precision
//! loss when working with 32-bit floats).  
//!
//! One possible gotcha: in OpenCV camera space, the coordinate system has the positive z
//! vector pointing into the frame and the positive y vector pointing
//! down from the top of the image.  Note that this is flipped
//! from OpenGL (where the _negative_ z vector points into the frame).  
//!
//! This class includes the \ref worldToCamera and \ref cameraToImage functions
//! for projecting a world space point down to the image plane.  Because inverting
//! camera distortion is considerably complicated, the inverses of 
//! these functions are not provided.  If you need undistortion functionality, 
//! you can find it within OpenCV; take a look at the 
//! <a href="http://docs.opencv.org/modules/imgproc/doc/geometric_transformations.html#undistortpoints">undistortPoints</a> function.
struct OpenCVCamera
{
    //! Initialize the camera model.
    OpenCVCamera (const size_t imageWidth_in, const size_t imageHeight_in,
                  const Transformd& extrinsics_in,
                  const double fx_in, const double fy_in,
                  const double cx_in, const double cy_in,
                  const double k1_in, const double k2_in, 
                  const double p1_in, const double p2_in,
                  const double k3_in);

    //! Initializes the camera to be a basic QVGA camera looking down the z axis.
    OpenCVCamera();

    //! \brief Computes the OpenGL modelview matrix corresponding to this camera.
    //!
    //! In most cases it should be used together with the result of \ref glProjectionMatrix.
    //! It can be passed to OpenGL using
    //! \code{.cpp}
    //!   const Matrix4d modelViewMat = camera.glModelViewMatrix();
    //!   glMatrixMode(GL_MODELVIEW);
    //!   glLoadIdentity();
    //!   glMultMatrixd(modelViewMat.data);
    //! \endcode
    Matrix4d glModelViewMatrix() const;

    //! \brief Computes the OpenGL projection matrix corresponding to this camera.
    //!
    //! In most cases it should be used together with the result of \ref glModelViewMatrix.
    //! It can be passed to OpenGL using
    //! \code{.cpp}
    //!   // Distances are in millimeters; 2m is a reasonable far plane:
    //!   const Matrix4d projMat = camera.glProjectionMatrix(10.0f, 2000.0f);
    //!   glMatrixMode(GL_PROJECTION);
    //!   glLoadIdentity();
    //!   glMultMatrixd(projectionMat.data);
    //! \endcode
    //! 
    //! \arg near_z The distance to the near z plane (in mm).
    //! \arg far_z The distance to the near z plane (in mm).
    Matrix4d glProjectionMatrix(double near_z, double far_z) const;

    //! \brief Takes a world space point and transforms it into camera space.  
    //!
    //! Camera space is the space aligned with the camera center
    //! where the positive z vector points into the frame.
    Vector3d worldToCamera (const Vector3d& p_world) const;

    //! \brief Takes a point in camera space and transforms it to the image plane.  
    //!
    //! In image space, x and y correspond to actual pixel values, measured from
    //! the top left corner of the image.  
    //!
    //! The z coordinate gets passed straight through from camera space without
    //! modification.  
    Vector3d cameraToImage (const Vector3d& p_camera) const;

    //! \brief Takes a world space point and projects it to the image plane.  
    //!
    //! Combines \ref worldToCamera with \ref cameraToImage.
    Vector3d project (const Vector3d& p_world) const { return cameraToImage (worldToCamera (p_world)); }

    //! \brief The transform taking desk to camera space.
    //!
    //! This is the camera extrinsics matrix which takes a point from desk space
    //! (where the origin is roughly at the center of the checkerboard you used to
    //! calibrate) to camera space (looking down the z axis).  
    Transformd extrinsics;

    //! The width of the depth image, in pixels.
    size_t imageWidth;

    //! The height of the depth image, in pixels.
    size_t imageHeight;

    /*! \name Intrinsics */

    //! x focal length (in pixels).
    double fx;
    //! y focal length (in pixels).
    double fy;

    /*! \name Distortion parameters */

    //! Camera center x coordinate (in pixels, from left).
    double cx;
    //! Camera center y coordinate (in pixels, from top).
    double cy;

    //! Radial distortion quadratic term.    
    double k1;
    //! Radial distortion r^4 term.
    double k2;
    //! Radial distortion r^6 term.
    double k3;

    //! Tangential distortion term.
    double p1;
    //! Tangential distortion term.
    double p2;
};

} // namespace HandTrackingClient

