#include <iostream>
#include <fstream>
#include <cstdarg>
#include <string>
#include <thread>
#include <chrono>
#include <conio.h>
#include <direct.h>
#include <wxWidgets64.h>

#pragma warning(disable:4471)

#include "Kinect.h"

#pragma comment(lib, "kinect20.lib")

using namespace std;

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
int fps = 1;

auto now() { return std::chrono::steady_clock::now(); }
 
auto awake_time()
{
    using std::chrono::operator""ns;
    return now() + 1000000000/fps*1ns;
}

ICoordinateMapper*    pCoordinateMapper    = nullptr;

void DrawSkeleton(const Joint* joints, wxImage& image)
{

    wxBitmap bitmap(image);
    wxMemoryDC dc(bitmap);

    dc.SetBrush(*wxRED_BRUSH);
    dc.SetPen(*wxRED_PEN);

    for (int j = 0; j < JointType_Count; ++j) {
        ColorSpacePoint colorPoint;
        pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);

        int x = static_cast<int>(colorPoint.X);
        int y = static_cast<int>(colorPoint.Y);

        if (x >= 0 && x < image.GetWidth() && y >= 0 && y < image.GetHeight())
        {

            for(int r = 1; r < 5; ++r)
            {
                dc.DrawCircle(x,y,r);
            }

            //wxNativePixelData::Iterator pixel(data);
            //pixel.Offset(data, x, y);
            //pixel.Red() = 255;
            //pixel.Green() = 0;
            //pixel.Blue() = 0;
        }
    }

    image = bitmap.ConvertToImage();
}


int main(int argc, char * argv[])
{
    wxImage::AddHandler(new wxBMPHandler);
    wxImage::AddHandler(new wxPNGHandler);
    wxImage::AddHandler(new wxTIFFHandler);
    wxImage::AddHandler(new wxJPEGHandler);

    if(argc > 1)
    {
        for(int i = 1; i < argc; ++i)
        {
            if (strncmp(argv[i],"fps:",4)==0) fps = atoi(argv[i] + 4);
        }
    }

    // Initialize
    IKinectSensor*        pSensor              = nullptr;
    IColorFrameReader*    pColorFrameReader    = nullptr;
    IBodyFrameReader*     pBodyFrameReader     = nullptr;
    IColorFrameSource*    pColorFrameSource    = nullptr;
    IBodyFrameSource*     pBodyFrameSource     = nullptr;

    // Initialize camera
    if (FAILED(GetDefaultKinectSensor(&pSensor)) || !pSensor) {
        std::cerr << "Kinect sensor initialization failed!" << std::endl;
        return -1;
    }
    if (FAILED(pSensor->Open())) {
        std::cerr << "Failed to open Kinect sensor!" << std::endl;
        return -1;
    }

    if (FAILED(pSensor->get_CoordinateMapper(&pCoordinateMapper))) {
        std::cerr << "Failed to get coordinate mapper!" << std::endl;
        return -1;
    }

    if (FAILED(pSensor->get_BodyFrameSource(&pBodyFrameSource))) {
        std::cerr << "Failed to get body frame source!" << std::endl;
        return -1;
    }

    if (FAILED(pBodyFrameSource->OpenReader(&pBodyFrameReader))) {
        std::cerr << "Failed to open body frame reader!" << std::endl;
        return -1;
    }

    // Initialize three sensors
    if (FAILED(pSensor->get_ColorFrameSource(&pColorFrameSource))) {
        std::cerr << "Failed to get color frame source!" << std::endl;
        return -1;
    }
    if (FAILED(pColorFrameSource->OpenReader(&pColorFrameReader))) {
        std::cerr << "Failed to open color frame reader!" << std::endl;
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

    for(int frame_no = 1, wait = 0;;)
    {

        if (wait == 1 || wait == 3)
        {
            awake = awake_time();
            wait = 2;
        }

        IColorFrame*    pColorFrame    = nullptr;
        IBodyFrame*     pBodyFrame     = nullptr;


        bool AcqColor = SUCCEEDED(pColorFrameReader->AcquireLatestFrame(&pColorFrame));
        bool AcqBody  = SUCCEEDED(pBodyFrameReader->AcquireLatestFrame(&pBodyFrame));
        if (AcqColor) cout << "c";
        if (AcqBody)  cout << "b";
        if (AcqColor && AcqBody)
        {
            string cfile = stringf("%s\\color_%02d.raw",dirname,frame_no);
            string bfile = stringf("%s\\body_%02d.jpg",dirname,frame_no);
            IFrameDescription* pFrameDescriptionC = nullptr;
            if (
                SUCCEEDED(pColorFrame->get_FrameDescription(&pFrameDescriptionC))
                //&& SUCCEEDED(pDepthFrame->get_FrameDescription(&pFrameDescriptionD))
                //&& SUCCEEDED(pInfraredFrame->get_FrameDescription(&pFrameDescriptionI))
               )
            {

                {

                    int colorWidth = 0, colorHeight = 0;
                    pFrameDescriptionC->get_Width(&colorWidth);
                    pFrameDescriptionC->get_Height(&colorHeight);

                    UINT bufferSize = colorWidth * colorHeight * 4;
                    BYTE* buffer = new BYTE[bufferSize];
                    ColorImageFormat imageFormat = ColorImageFormat_Bgra;
                    pColorFrame->get_RawColorImageFormat(&imageFormat);
                    if (imageFormat == ColorImageFormat_Bgra) {
                        pColorFrame->CopyRawFrameDataToArray(bufferSize, buffer);
                    } else {
                        pColorFrame->CopyConvertedFrameDataToArray(bufferSize, buffer, ColorImageFormat_Bgra);
                    }

                    unsigned char * s = (unsigned char *)malloc(3*colorWidth*colorHeight);
                    for(int i = 0; i < colorWidth*colorHeight; ++i)
                    {
                        s[3*i]   = buffer[4*i+2];
                        s[3*i+1] = buffer[4*i+1];
                        s[3*i+2] = buffer[4*i+0];
                    }

                    wxImage colorImage(colorWidth, colorHeight,s);


                    IBody* ppBodies[BODY_COUNT] = { 0 };
                    if (SUCCEEDED(pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies))) {
                        for (int i = 0; i < BODY_COUNT; ++i) {
                            IBody* pBody = ppBodies[i];
                            if (pBody) {
                                BOOLEAN bTracked = false;
                                if (SUCCEEDED(pBody->get_IsTracked(&bTracked)) && bTracked) {
                                    Joint joints[JointType_Count];
                                    if (SUCCEEDED(pBody->GetJoints(_countof(joints), joints))) {
                                        DrawSkeleton(joints, colorImage);
                                    }
                                }
                            }
                        }
                    }

                    cout << "Color image " << frame_no << "  " << colorWidth << " x " << colorHeight << endl;
                    colorImage.Mirror().SaveFile(bfile.c_str());

                    delete[] buffer;
                }

                if (1) {// Color image
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
                frame_no++;
                //cout << "Frame++ = " << frame_no << endl;
                if (wait == 0) wait = 3; else wait = 1;
                //cout << "Wait = " << wait << endl;
            }
            if (pFrameDescriptionC) pFrameDescriptionC->Release();
        }

        if (pColorFrame)    pColorFrame->Release();
        if (pBodyFrame)     pBodyFrame->Release();
        if (_kbhit() && _getch() == 0x1B) break;

        if (wait == 1)
        {
            this_thread::sleep_until(awake);
        }

    }

    // Housekeeping
    if (pColorFrameReader) pColorFrameReader->Release();
    if (pBodyFrameReader)  pBodyFrameReader->Release();
    if (pSensor) pSensor->Close();
    if (pSensor) pSensor->Release();

    return 0;
}
