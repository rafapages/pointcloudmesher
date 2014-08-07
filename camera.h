#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>

class Camera{

public:
    Camera();
    ~Camera();

    float getFocalLenght();
    Eigen::Matrix3f getRotationMatrix();
    Eigen::Vector3f getTranslationVector();

    void setFocalLenght(const float _f);
    void setDistortionCoefficients(const float _k1, const float _k2);
    void setRotationMatrix(const Eigen::Matrix3f _R);
    void setTranslationVector(const Eigen::Vector3f _t);

    Eigen::Vector3f getCameraPosition();


private:
    float f_; // Focal lenght
    float k1_, k2_; // Radial distortion coefficients
    Eigen::Matrix3f R_; // Rotation matrix
    Eigen::Vector3f t_; // Camera translation vector

};



#endif
