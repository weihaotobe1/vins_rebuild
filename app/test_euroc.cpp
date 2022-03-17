//
// Created by weihao on 2022/3/16.
//

#include "system.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "global_param.hpp"
#include <string>

using namespace std;
vinssystem mSystem;

/// Load EuRoC DataSet
/**
 * @brief load_image_data EuRoC
 * @param dataset_path
 * @param image_data
 */
static void LoadImageData(const string dataset_path, vector<pair<double, string> > &image_data)
{
    string path = dataset_path + "cam0/data.csv";
    ifstream fin;
    fin.open(path);
    if(!fin.is_open())
    {
        std::cerr << "------------Reading image data error--------------" << std::endl;
        return;
    }

    // Skip the first line
    string str0;
    getline(fin, str0);

    while(!fin.eof())
    {
        string str;
        getline(fin, str);

        if(!str.empty())
        {
            // Replace all "," for " "
            size_t pos = str.find(",");
            while(pos != str.npos)
            {
                str.replace(pos, 1, " ");
                pos = str.find(",", pos+1);
            }

            stringstream ss;
            ss << str;

            double timestamp;
            string img_name;
            ss >> timestamp;
            ss >> img_name;

            timestamp = timestamp/1e9;

            image_data.push_back(std::make_pair(timestamp, img_name));
        }
    }

    fin.close();
}


/**
 * @brief load_imu_data
 * @param dataset_path
 * @param imu_data
 */
static void LoadImuData(const string dataset_path, vector<Eigen::Matrix<double, 7, 1> > &imu_data)
{
    string path = dataset_path + "imu0/data.csv";
    ifstream fin;
    fin.open(path);
    if(!fin.is_open())
    {
        std::cerr << "------------Reading IMU data error--------------" << std::endl;
        return;
    }

    // Skip the first line
    string str0;
    getline(fin, str0);

    while(!fin.eof())
    {
        string str;
        getline(fin, str);

        if(!str.empty())
        {
            // Replace all "," for " "
            size_t pos = str.find(",");
            while(pos != str.npos)
            {
                str.replace(pos, 1, " ");
                pos = str.find(",", pos+1);
            }

            stringstream ss;
            ss << str;

            double timestamp, wx, wy, wz, ax, ay, az;
            ss >> timestamp;
            ss >> wx; ss >> wy; ss >> wz;
            ss >> ax; ss >> ay; ss >> az;

            Eigen::Matrix<double, 7, 1> data;
            data << timestamp/1e9, wx, wy, wz, ax, ay, az;

            imu_data.push_back(data);
        }
    }

    fin.close();
}

void IMUProc(Eigen::Matrix<double, 7, 1>& imu_data, vinssystem* mpSystem)
{
    ImuConstPtr imu_msg = new IMU_MSG();
    imu_msg->header = imu_data(0);
    imu_msg->acc(0) = imu_data(4);
    imu_msg->acc(1) = imu_data(5);
    imu_msg->acc(2) = imu_data(6);
    imu_msg->gyr(0) = imu_data(1);
    imu_msg->gyr(1) = imu_data(2);
    imu_msg->gyr(2) = imu_data(3);

    mpSystem->inputIMU(imu_msg);
}

void ImageProc(cv::Mat srcImage, double header, vinssystem* mpSystem)
{
    mpSystem->inputImage(srcImage,header);
    return;
}

/// 主函数
int main(int argc, char** argv){

//    if(argc != 3)
//    {
//        cerr << endl << "Usage: test_euroc path_of_config path_of_dataset" << endl;
//        cout<<"eg: test_euroc ../config/euroc.yaml "<<endl;
//        return 1;
//    }

    string ssettingfile = string(argv[1]);

    mSystem.create(ssettingfile);

    for(int i = 2; i < argc; i++) // 刘大神这个操作估计是为了多数据集测试
    {
        string sdatafolder;

        sdatafolder = string(argv[i]);

        cout << "=============Loading IMU and image data============" << endl;
        vector<pair<double, string>> image_datas;//加载图像信息
        vector<Eigen::Matrix<double, 7, 1> > imu_datas;
        LoadImageData(sdatafolder, image_datas);
        std::cout << "Image size : " << image_datas.size() << std::endl;
        LoadImuData(sdatafolder, imu_datas);//加载IMU信息
        std::cout << "IMU size : " << imu_datas.size() << std::endl;


        while(image_datas.size() && imu_datas.size())
        {

            if(imu_datas[0](0) < image_datas[0].first)
            {
                IMUProc(imu_datas[0],&mSystem);
                imu_datas.erase(imu_datas.begin());
                usleep(2000);
            }
            else
            {
                std::string image_path = sdatafolder + "cam0/data/" + image_datas[0].second;
                cv::Mat srcImage = cv::imread(image_path,CV_LOAD_IMAGE_GRAYSCALE);

                ImageProc(srcImage.clone(),image_datas[0].first,&mSystem);

                image_datas.erase(image_datas.begin());

                cout << "Images Left" << image_datas.size() << endl;

                usleep(29000);
            }
        }
    }

    return 0;
}

