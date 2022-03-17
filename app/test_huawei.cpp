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

/// Load HUAWEI P30 DataSet
/**
 * @brief load_image_data
 * @param dataset_path
 * @param image_data
 */
static void LoadImageDataP30Pro(const string dataset_path, vector<pair<double, string> > &image_data)
{
    string path = dataset_path + "IMU/img.txt";
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
            stringstream ss;
            ss << str;

            string timestamp_str;
            ss >> timestamp_str;
            string img_name = timestamp_str + ".YUV";

            double timestamp = atof(timestamp_str.c_str());
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
static void LoadImuDataP30Pro(const string dataset_path, vector<Eigen::Matrix<double, 7, 1> > &imu_data)
{
    string path_gyro = dataset_path + "IMU/G.txt";
    string path_acc = dataset_path + "IMU/A.txt";
    ifstream fin_gyro;
    fin_gyro.open(path_gyro);
    ifstream fin_acc;
    fin_acc.open(path_acc);
    if(!fin_gyro.is_open() && !fin_acc.is_open())
    {
        std::cerr << "------------Reading IMU data error--------------" << std::endl;
        return;
    }


    while(!fin_gyro.eof() && !fin_acc.eof())
    {
        string str_gyro;
        getline(fin_gyro, str_gyro);

        string str_acc;
        getline(fin_acc, str_acc);

        if(!str_gyro.empty() && !str_acc.empty())
        {

            stringstream ss;
            ss << str_gyro;

            double timestamp, wx, wy, wz;
            ss >> timestamp;
            ss >> wx; ss >> wy; ss >> wz;

            stringstream ss_a;
            ss_a << str_acc;
            double timestamp_a, ax, ay, az;
            ss_a >> timestamp_a;
            ss_a >> ax; ss_a >> ay; ss_a >> az;

            Eigen::Matrix<double, 7, 1> data;
            data << timestamp/1e9, wx, wy, wz, ax, ay, az;

            imu_data.push_back(data);
        }
    }

    fin_gyro.close();
    fin_acc.close();
}


/**
 * @brief Load YUV image
 * @param img_path: path of the image file
 * @param width:    image width
 * @param height:   image height
 * */
static cv::Mat LoadYuvImg(const string& img_path, const int width, const int height)
{
    FILE* fp_in = fopen(img_path.c_str(), "rb");
    uchar *yuv_data = new uchar[height * 3 / 2 * width * sizeof(uchar)];
    size_t res = fread(yuv_data, height * 3 / 2, width, fp_in);
    if(res == 0)
    {
        std::cerr << "Reading YUV Error!" << std::endl;
    }

    cv::Mat yuv_img(height * 3 / 2, width, CV_8UC1, yuv_data);
    cv:: Mat rgb_img(height, width, CV_8UC3);
    cv::cvtColor(yuv_img, rgb_img, CV_YUV2BGR_NV12);

    fclose(fp_in);
    delete[] yuv_data;

    return rgb_img;
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
//        cerr << endl << "Usage: test_huawei path_of_config path_of_dataset" << endl;
//        cout<<"eg: test_euroc ../config/huawei.yaml "<<endl;
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
        LoadImageDataP30Pro(sdatafolder, image_datas);
        std::cout << "Image size : " << image_datas.size() << std::endl;
        LoadImuDataP30Pro(sdatafolder, imu_datas);//加载IMU信息
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
                cv::Mat srcImage;
                std::string image_path = sdatafolder + "YUV/" + image_datas[0].second;
                srcImage = LoadYuvImg(image_path, 640, 480);

                if (srcImage.empty())
                {
                    cerr << "------ Failed to load image ------" << endl;
                    break;
                }
                if(srcImage.channels() != 1) // convert color image to gray
                    cv::cvtColor(srcImage, srcImage, cv::COLOR_RGB2GRAY);

                ImageProc(srcImage.clone(),image_datas[0].first,&mSystem);

                image_datas.erase(image_datas.begin());

                cout << "Images Left" << image_datas.size() << endl;

                usleep(29000);
            }
        }
    }

    return 0;
}

