#include <iostream>
#include <fstream>
#include <string>

#include <boost/tokenizer.hpp>

#include "camera.h"

Camera::Camera(){

}

Camera::~Camera(){

}

float Camera::getFocalLength(){
    return f_;
}

Eigen::Matrix3f Camera::getRotationMatrix(){
    return R_;
}

Eigen::Vector3f Camera::getTranslationVector(){
    return t_;
}

void Camera::setFocalLength(const float _f){
    f_ = _f;
}

void Camera::setDistortionCoefficients(const float _k1, const float _k2){
    k1_ = _k1;
    k2_ = _k2;
}

void Camera::setRotationMatrix(const Eigen::Matrix3f _R){
    R_ = _R;
}

void Camera::setTranslationVector(const Eigen::Vector3f _t){
    t_ = _t;
}

Eigen::Vector3f Camera::getCameraPosition(){ // -R'·t
    return -R_.transpose() * t_;
}

void Camera::getCameraPosition(PointXYZRGBNormalCam& _point){
    Eigen::Vector3f pos = this->getCameraPosition();
    _point.x = pos(0);
    _point.y = pos(1);
    _point.z = pos(2);
}

// NOTE: IT'S NECESSARY TO ADD COMMENT PROTECTION TO THIS METHOD (#)
void Camera::readCamera(std::ifstream& _stream){
    Eigen::Matrix3f R;
    std::stringstream ss;
    std::string line;

    for (unsigned int cam_line = 0; cam_line < 5; cam_line++){

        std::getline(_stream, line);
        while (line.empty()) std::getline(_stream, line);


        boost::tokenizer<boost::char_separator<char> > cam_tokens(line, boost::char_separator<char>(" "));
        boost::tokenizer<boost::char_separator<char> >::iterator ctit = cam_tokens.begin();

        // First line has focal lenght and distortion coefficients
        if (cam_line == 0){
            float f, k1, k2;
            ss << *ctit;
            ss >> f;
            ++ctit;
            ss.str(std::string());
            ss.clear();
            ss << *ctit;
            ss >> k1;
            ++ctit;
            ss.str(std::string());
            ss.clear();
            ss << *ctit;
            ss >> k2;
            ss.str(std::string());
            ss.clear();
            this->setFocalLength(f);
            this->setDistortionCoefficients(k1,k2);
        }
        // Last line has the translation vector
        else if (cam_line == 4){
            Eigen::Vector3f t;
            unsigned int index = 0;
            for (; ctit != cam_tokens.end(); ++ctit, index++){
                float value;
                ss << *ctit;
                ss >> value;
                ss.str(std::string());
                ss.clear();
                t(index) = value;
            }
            this->setTranslationVector(t);
        }
        // The other lines have the rotation matrix
        else {
            unsigned int index = 0;
            for (; ctit != cam_tokens.end(); ++ctit, index++){
                float value;
                ss << *ctit;
                ss >> value;
                ss.str(std::string());
                ss.clear();
                R(cam_line-1, index) = value;
            }
        }

    }
    this->setRotationMatrix(R);
}
