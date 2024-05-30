#include <iostream>
#include <fstream>
#include <cstdarg>
#include <string>
#include <thread>
#include <chrono>
#include <conio.h>
#include <direct.h>

#pragma warning(disable:4471)

#include "Kinect.h"

#pragma comment(lib, "kinect20.lib")

using namespace std;
using namespace std::chrono;

string stringf(const char * fmt, ...)
{
    va_list args;
    va_start(args,fmt);
    int len = vsnprintf(nullptr,0,fmt,args);
    string s(len,' ');
    vsnprintf(s.data(),len+1,fmt,args);
    va_end(args);
    return s;
}

int every_frame = 1;   // every ... frame
int fps = 10;

auto now() { return std::chrono::steady_clock::now(); }
 
auto awake_time()
{
    using std::chrono::operator""ns;
    return now() + 1000000000/fps*1ns;
}

int main(int argc, char * argv[])
{
    if(argc > 1)
    {
        for(int i = 1; i < argc; ++i)
        {
            if (strncmp(argv[i],"fps:",4)==0) fps = atoi(argv[i] + 4);
        }
    }

    // Initialize
    IKinectSensor*        pSensor = nullptr;
    IColorFrameReader*    pColorFrameReader = nullptr;
    IDepthFrameReader*    pDepthFrameReader = nullptr;
    IInfraredFrameReader* pInfraredFrameReader = nullptr;

    // Initialize camera
    if (FAILED(GetDefaultKinectSensor(&pSensor)) || !pSensor) {
        std::cerr << "Kinect sensor initialization failed!" << std::endl;
        return -1;
    }
    if (FAILED(pSensor->Open())) {
        std::cerr << "Failed to open Kinect sensor!" << std::endl;
        return -1;
    }
    // Initialize three sensors
    IColorFrameSource* pColorFrameSource = nullptr;
    if (FAILED(pSensor->get_ColorFrameSource(&pColorFrameSource))) {
        std::cerr << "Failed to get color frame source!" << std::endl;
        return -1;
    }
    if (FAILED(pColorFrameSource->OpenReader(&pColorFrameReader))) {
        std::cerr << "Failed to open color frame reader!" << std::endl;
        return -1;
    }

    IDepthFrameSource* pDepthFrameSource = nullptr;
    if (FAILED(pSensor->get_DepthFrameSource(&pDepthFrameSource))) {
        std::cerr << "Failed to get depth frame source!" << std::endl;
        return -1;
    }
    if (FAILED(pDepthFrameSource->OpenReader(&pDepthFrameReader))) {
        std::cerr << "Failed to open depth frame reader!" << std::endl;
        return -1;
    }

    IInfraredFrameSource* pInfraredFrameSource = nullptr;
    if (FAILED(pSensor->get_InfraredFrameSource(&pInfraredFrameSource))) {
        std::cerr << "Failed to get infrared frame source!" << std::endl;
        return -1;
    }
    if (FAILED(pInfraredFrameSource->OpenReader(&pInfraredFrameReader))) {
        std::cerr << "Failed to open infrared frame reader!" << std::endl;
        return -1;
    }

    // Add subdirectory for frames in format YYYYMMDD_hhmmss
    time_t t = time(nullptr);
    tm tm;
    localtime_s(&tm,&t);
    char dirname[16];
    strftime(dirname, sizeof(dirname), "%Y%m%d_%H%M%S", &tm);

    _mkdir(dirname);


    decltype(awake_time()) awake;
    decltype(now()) start, stop;

    for(int frame_no = 1, check = 0, wait = 0;;)
    {

        if (wait == 1 || wait == 3)
        {
            awake = awake_time();
            wait = 2;
        }

        IColorFrame*    pColorFrame = nullptr;
        IDepthFrame*    pDepthFrame = nullptr;
        IInfraredFrame* pInfraredFrame = nullptr;

        if (  // Need ALL frames
            SUCCEEDED(pColorFrameReader->AcquireLatestFrame(&pColorFrame)) &&
            SUCCEEDED(pDepthFrameReader->AcquireLatestFrame(&pDepthFrame)) &&
            SUCCEEDED(pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame)) &&
            ++check%every_frame == 0
           )
        {
            string cfile = stringf("%s\\color_%02d.raw",dirname,frame_no);
            string dfile = stringf("%s\\depth_%02d.raw",dirname,frame_no);
            string ifile = stringf("%s\\infra_%02d.raw",dirname,frame_no);
            IFrameDescription* pFrameDescriptionC = nullptr;
            IFrameDescription* pFrameDescriptionD = nullptr;
            IFrameDescription* pFrameDescriptionI = nullptr;
            if (
                SUCCEEDED(pColorFrame->get_FrameDescription(&pFrameDescriptionC)) &&
                SUCCEEDED(pDepthFrame->get_FrameDescription(&pFrameDescriptionD)) &&
                SUCCEEDED(pInfraredFrame->get_FrameDescription(&pFrameDescriptionI)))
            {
                if (wait == 0)
                {
                    wait = 3;
                    awake = awake_time();
                    start = now();
                }
                else wait = 1;
                {// Color image
                    int width = 0, height = 0;
                    pFrameDescriptionC->get_Width(&width);
                    pFrameDescriptionC->get_Height(&height);
                    UINT bufferSize = width * height * 4;
                    BYTE* buffer = new BYTE[bufferSize];
                    cout << "Color frame " << frame_no << "  " << width << " x " << height << endl;
                    ColorImageFormat imageFormat = ColorImageFormat_Bgra;
                    pColorFrame->get_RawColorImageFormat(&imageFormat);
                    if (imageFormat == ColorImageFormat_Bgra) {
                        pColorFrame->CopyRawFrameDataToArray(bufferSize, buffer);
                    } else {
                        pColorFrame->CopyConvertedFrameDataToArray(bufferSize, buffer, ColorImageFormat_Bgra);
                    }
                    std::ofstream colorFile(cfile.c_str(), std::ios::binary);
                    colorFile.write(reinterpret_cast<char*>(buffer), bufferSize);
                    colorFile.close();
                    delete[] buffer;
                }
                { // Depth image
                    int width = 0, height = 0;
                    pFrameDescriptionD->get_Width(&width);
                    pFrameDescriptionD->get_Height(&height);

                    UINT16* pDepthBuffer = new UINT16[width * height];
                    pDepthFrame->CopyFrameDataToArray(width * height, pDepthBuffer);

                    cout << "Depth frame " << frame_no << "  " << width << " x " << height << endl;

                    std::ofstream depthFile(dfile.c_str(), std::ios::binary);
                    depthFile.write(reinterpret_cast<char*>(pDepthBuffer), width * height * sizeof(UINT16));
                    depthFile.close();
                    delete[] pDepthBuffer;
                }
                { // Infrared frame
                    int width = 0, height = 0;
                    pFrameDescriptionI->get_Width(&width);
                    pFrameDescriptionI->get_Height(&height);

                    cout << "infra frame " << frame_no << "  " << width << " x " << height << endl;

                    UINT16* pInfraredBuffer = new UINT16[width * height];
                    pInfraredFrame->CopyFrameDataToArray(width * height, pInfraredBuffer);

                    std::ofstream infraredFile(ifile.c_str(), std::ios::binary);
                    infraredFile.write(reinterpret_cast<char*>(pInfraredBuffer), width * height * sizeof(UINT16));
                    infraredFile.close();
                    delete[] pInfraredBuffer;
                }

                frame_no++;
            }
            if (pFrameDescriptionC) pFrameDescriptionC->Release();
            if (pFrameDescriptionD) pFrameDescriptionD->Release();
            if (pFrameDescriptionI) pFrameDescriptionI->Release();
        }

        if (pColorFrame)    pColorFrame->Release();
        if (pDepthFrame)    pDepthFrame->Release();
        if (pInfraredFrame) pInfraredFrame->Release();

        if (wait == 1 || wait == 3)
        {
            this_thread::sleep_until(awake);
        }
        if (_kbhit() && _getch() == 0x1B)
        {
            stop = now();
            auto duration = duration_cast<milliseconds>(stop-start);
            cout << "Average " << fixed << setprecision(1) << frame_no*1000.0/duration.count() << " fps\n";
            break;
        }

    }

    // Housekeeping
    if (pColorFrameReader) pColorFrameReader->Release();
    if (pDepthFrameReader) pDepthFrameReader->Release();
    if (pSensor) pSensor->Close();
    if (pSensor) pSensor->Release();

    return 0;
}
