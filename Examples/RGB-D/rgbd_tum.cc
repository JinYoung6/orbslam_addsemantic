/**
 * @file rgbd_tum.cc
 * @author guoqing (1337841346@qq.com)
 * @brief TUM RGBD 数据集上测试ORB-SLAM2
 * @version 0.1
 * @date 2019-02-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */


/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>
#include<opencv2/core/core.hpp>

#include <memory>

#include<System.h>

#include "Frame.h"
#include "Object.h"

using namespace std;

/**
 * @brief 加载图像
 * 
 * @param[in] strAssociationFilename    关联文件的访问路径
 * @param[out] vstrImageFilenamesRGB     彩色图像路径序列
 * @param[out] vstrImageFilenamesD       深度图像路径序列
 * @param[out] vTimestamps               时间戳
 */
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadBoundingBox(const string& strPathToDetectionResult, vector<std::pair<vector<double>, int>>& detect_result);

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_detectResult" << endl;
        return 1;
    }

    // Retrieve paths to images
    //按顺序存放需要读取的彩色图像、深度图像的路径，以及对应的时间戳的变量
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    //从命令行输入参数中得到关联文件的路径
    string strAssociationFilename = string(argv[4]);
    //从关联文件中加载这些信息
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    //彩色图像和深度图像数据的一致性检查
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //初始化ORB-SLAM2系统
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    vector<std::pair<vector<double>, int>> detect_result;
    //对图像序列中的每张图像展开遍历
    // Object obj;
    
    for(int ni=0; ni<nImages; ni++)
    {
        //****
        string strPathToDetectionResult = argv[5] + std::to_string(vTimestamps[ni]) + ".txt";//读取detect_result
        LoadBoundingBox(strPathToDetectionResult, detect_result);
        if (detect_result.empty()){
            cerr << endl << "LoadBoundingBox is wrong !" << endl;
            return 1;
        }
        //*****
        
        //! 读取图像
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        //! 确定图像合法性
        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        //test:在图像上添加一个检测框
        // cv::Mat imRGB2;
        // imRGB2 = imRGB.clone();
        // for (int k=0; k<detect_result.size(); ++k){
        //     if (detect_result[k].second == 3) {
        //         cv::Point pt1 = cv::Point(detect_result[k].first[0],detect_result[k].first[1]);
        //         cv::Point pt2 = cv::Point(detect_result[k].first[2],detect_result[k].first[3]);
        //         // cv::rectangle(imRGB2,pt1,pt2,cvScalar(0,255,0));
        //         // cv::line(imRGB,pt1,pt2,cvScalar(255,255,255));
        //     }
        // }
        // // cv::imshow("im2",imRGB2);
        // // sleep(1);





#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    
        // Pass the image to the SLAM system
        //! 追踪
        SLAM.TrackRGBD(imRGB,imD,tframe,detect_result);
        detect_result.clear();

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        //! 计算耗时
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        //! 根据时间戳,准备加载下一张图片
        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    //终止SLAM过程
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    //统计分析追踪耗时
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    //保存最终的相机轨迹
    //SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveTrajectoryTUM("./TUM_fr3_walking_half_results/CameraTrajectory1.txt");

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

//从关联文件中提取这些需要加载的图像的路径和时间戳
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    //输入文件流
    ifstream fAssociation;
    //打开关联文件
    fAssociation.open(strAssociationFilename.c_str());
    //一直读取,知道文件结束
    while(!fAssociation.eof())
    {
        string s;
        //读取一行的内容到字符串s中
        getline(fAssociation,s);
        //如果不是空行就可以分析数据了
        if(!s.empty())
        {
            //字符串流
            stringstream ss;
            ss << s;
            //字符串格式:  时间戳 rgb图像路径 时间戳 深度图像路径
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

void LoadBoundingBox(const string& strPathToDetectionResult, vector<std::pair<vector<double>, int>>& detect_result){
    ifstream infile;
    infile.open(strPathToDetectionResult);
    if (!infile.is_open()) {
        cout<<"yolo_detection file open fail"<<endl;
        exit(233);
    }
    vector<double> result_parameter;
    string line;
    while (getline(infile, line)){
        int sum = 0, num_bit = 0;
        for (char c : line) {//读取数字.    例如读取"748",先读7,再7*10+8=78,再78*10+4,最后读到空格结束
            if (c >= '0' && c <= '9') {
                num_bit = c - '0';
                sum = sum * 10 + num_bit;
            } else if (c == ' ') {
                result_parameter.push_back(sum);
                sum = 0;
                num_bit = 0;
            }
        }

        string idx_begin = "class:";//读取物体类别
        int idx = line.find(idx_begin);
        string idx_end = "0.";
        int idx2 = line.find(idx_end);
        string class_label;
        for (int j = idx + 6; j < idx2-1; ++j){
            class_label += line[j];
        }
        // cout << "**" << class_label << "**";

        int class_id = -1;//存入识别物体的种类
        if (class_label == "person") { //高动态物体:人,动物等
            class_id = 3;
        }

        if (class_label == "tv" ||   //低动态物体(在程序中可以假设为一直静态的物体):tv,refrigerator
            class_label == "refrigerator" || 
            class_label == "teddy bear") {
            class_id = 1;
        }

        if (class_label == "chair" || //中动态物体,在程序中不做先验动态静态判断
            class_label == "car"){
            class_id =2;
        }

        detect_result.emplace_back(result_parameter,class_id);
        result_parameter.clear();
        line.clear();
    }
    infile.close();

}