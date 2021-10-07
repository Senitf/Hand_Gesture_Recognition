#include "HandTrackingClient/OpenCVCamera.h"
#include <cassert>

namespace HandTrackingClient
{

OpenCVCamera::OpenCVCamera (const size_t imageWidth_in, const size_t imageHeight_in,
                            const Transformd& extrinsics_in,
                            const double fx_in, const double fy_in,
                            const double cx_in, const double cy_in,
                            const double k1_in, const double k2_in, 
                            const double p1_in, const double p2_in,
                            const double k3_in)
    : extrinsics (extrinsics_in),
      imageWidth (imageWidth_in), imageHeight (imageHeight_in),
      fx(fx_in), fy(fy_in),
      cx(cx_in), cy(cy_in),
      k1(k1_in), k2(k2_in), k3(k3_in),
      p1(p1_in), p2(p2_in)
{
}

const size_t QVGA_WIDTH = 320;
const size_t QVGA_HEIGHT = 240;
const double KINECT_FOCAL_LENGTH = 575.815753;

OpenCVCamera::OpenCVCamera()
    : imageWidth (QVGA_WIDTH), imageHeight (QVGA_HEIGHT),
      fx(KINECT_FOCAL_LENGTH), fy(KINECT_FOCAL_LENGTH),
      cx(QVGA_WIDTH/2), cy(QVGA_HEIGHT/2),
      k1(0), k2(0), k3(0),
      p1(0), p2(0)
{
}

Matrix4d 
OpenCVCamera::glModelViewMatrix() const
{
    Matrix4d result (extrinsics.rotation, extrinsics.translation);

    // In OpenCV, the y axis points down and the positive z axis points into the camera.
    // In OpenGL, the y axis points up and the negative z axis points into the camera.
    // To get from one to the other requires a 180 degree rotation about x, which we
    // can do by simultaneously flipping the y and z coordinates.
    for (unsigned jCol = 0; jCol < 4; ++jCol)
        result(1, jCol) *= -1.0;

    for (unsigned jCol = 0; jCol < 4; ++jCol)
        result(2, jCol) *= -1.0;

    return result;
}

Matrix4d 
OpenCVCamera::glProjectionMatrix(double zNear, double zFar) const
{
    assert (zFar > zNear);

    const double w = (double) imageWidth;
    const double h = (double) imageHeight;

    Matrix4d projectionMatrix;
    projectionMatrix(0, 0) = 2.0 * fx / w;
    projectionMatrix(1, 1) = 2.0 * fy / h;

    // TODO I'm not sure why one of these is inverted from the other:
    projectionMatrix(0, 2) = -2.0 * cx / w + 1.0;
    projectionMatrix(1, 2) =  2.0 * cy / h - 1.0;

    projectionMatrix(2, 2) = (zFar + zNear) / (zNear - zFar);
    projectionMatrix(3, 2) = -1.0;

    projectionMatrix(2, 3) = (2.0 * zFar * zNear) / (zNear - zFar);

    return projectionMatrix;
}

Vector3d
OpenCVCamera::worldToCamera (const Vector3d& p_world) const
{
    return extrinsics * p_world;
}

Vector3d 
OpenCVCamera::cameraToImage (const Vector3d& p_camera) const
{
    // Project:
    const double xp = p_camera.x / p_camera.z;
    const double yp = p_camera.y / p_camera.z;

    const double rSqr = xp*xp + yp*yp;
    const double rFourth = rSqr*rSqr;
    const double rSixth = rFourth*rSqr;

    // OpenCV distortion model;
    // see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    const double radialDistortion = (1.0 + k1*rSqr + k2*rFourth + k3*rSixth);
    const double xpp = xp * radialDistortion + 
                       2.0 * p1 * xp * yp +
                       p2 * (rSqr + 2.0 * xp * xp);
    const double ypp = yp * radialDistortion +
                       p1 * (rSqr + 2.0 * yp * yp) + 
                       2.0 * p2 * xp * yp;

    const double u = fx * xpp + cx;
    const double v = fy * ypp + cy;

    return Vector3d (u, v, p_camera.z);
}

} // namespace HandTrackingClient
