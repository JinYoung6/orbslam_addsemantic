//
//Created by jy on 21-3-28.
//
#ifndef MYSLAM_OBJECT_H
#define MYSLAM_OBJECT_H

#include <fstream>
#include <mutex>
#include <set>
#include "MapPoint.h"
using namespace std;

namespace ORB_SLAM2 {
    class MapPoint;
    class Object{
        public:
        Object();
        Object(vector<double> vdetect_parameter_,int ndetect_class_);
        ~Object();

        public:
        enum classname{
            person = 3,
            car = 1
        };

        public:
        vector<double> vdetect_parameter;//检测框的四个参数
        int ndetect_class;
        bool bdynamic_;

        public:

        vector<double> GetDetectParameter();
        int GetDetectClass();

    };
}

#endif 