#include <stdio.h>
#include <string.h>
#include <sl/Camera.hpp>
#include <opencv2/core/core.hpp>;
#include <opencv2/highgui/highgui.hpp>;
#include <opencv2/imgproc/imgproc.hpp>;

#define FOCAL_LENGTH 100;
#define PIXEL_SIZE 0.004;

using namespace sl;
using namespace std;

void display_volumn(sl::Mat depth_map, cv::Mat mask, vector<pair<cv::Point, cv::Point>> description) {

    float min_distance = 99;
    float max_distance = 0;
    float volumn = 0;
    float surface_area = 0;
    int pixelcount = 0;
    int objectnum = 1;
    float depth_value = 0;

    

    while (!description.empty()) {
        auto iter = description.back();
        std::cout << "\n" << "\n";
        std::cout << "Analyzing object "<< objectnum << "\n";
        volumn = 0;
        min_distance = 99;
        max_distance = 0;
        volumn = 0;
        surface_area = 0;
        pixelcount = 0;
        depth_value = 0;

        for (int i = iter.first.x; i < iter.second.x; i++) {
            for (int j = iter.first.y; j < iter.second.y; j++) {
                if (mask.at<cv::Vec3b>(i, j) != cv::Vec3b(0, 0, 0)) {
                    pixelcount++;
                    depth_map.getValue(i, j, &depth_value);
                    if (min_distance > depth_value && depth_value >= 0.15) min_distance = depth_value;
                    if (max_distance < depth_value && max_distance < 2 && depth_value >= 0.15) max_distance = depth_value;
                }
            }
        }
        min_distance = min_distance - 0.18;
        float vertical_realdistance = 2 * float(min_distance) * 0.7265;
        float horizontal_realdistance = 2 * float(min_distance) * 1.2799;
        float distance_perPixelv = vertical_realdistance / 720;
        float distance_perPixelh = horizontal_realdistance / 1280;
        printf("distance from len to object is %f\n", min_distance);
        printf("distance per pixel x = %f\n", distance_perPixelv);
        printf("distance per pixel y = %f\n", distance_perPixelh);
        printf("total area pixel count = %d\n", pixelcount);
        surface_area = distance_perPixelh * distance_perPixelv * pixelcount;
        printf("real surface area of the detected object is: %f square meters\n", surface_area);
        volumn = powl(surface_area, 1.5) / 10.635;
        printf("volumn of the detected object is %f cubic meters\n", volumn);
        objectnum++;
        description.pop_back();
    }

    return;
}

// Need to get the mask matrix for the picture;
int main() {
    Camera zed;
    sl::InitParameters init_parameters;
    sl::Mat image;
    sl::Mat depth_map;
    float objectvolumn = 0;
    std::string object = "charger";

    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.depth_mode = DEPTH_MODE::QUALITY;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.depth_minimum_distance = float(0.10) ; // Set the minimum depth perception distance to 10cm

    ERROR_CODE zed_error = zed.open(init_parameters);

    // Check camera;
    if (zed_error != ERROR_CODE::SUCCESS) { 
        printf("Opening camera failed");
        zed.close();
        return 1;
    }

    //Get image and retrieve map;
    if (zed.grab() == ERROR_CODE::SUCCESS) {
        zed.retrieveImage(image, VIEW::LEFT);
        zed.retrieveMeasure(depth_map,MEASURE::DEPTH);
        printf("Image resolution: %d x %d\n", (int)image.getWidth(), (int)image.getHeight());
    }
    else {
        printf("Unable to grab image!");
        zed.close();
        return -1;
    }
    
    image.write("./output.jpg");

    cv::Mat srcImage = cv::imread("./output.jpg", CV_LOAD_IMAGE_COLOR);

    if (!srcImage.data)                              // Check for invalid input
    {
       cout << "Could not open or find the image" << std::endl;
       return -1;
    }

    std::vector<pair<cv::Point, cv::Point>> description;

    cv::Mat mask = cv::Mat::zeros(srcImage.size(), srcImage.type());
    cv::Mat dstImage = cv::Mat::zeros(srcImage.size(), srcImage.type());

    //Establish mask and bounding box; 
    cv::circle(mask, cv::Point(srcImage.cols/2, srcImage.rows/2 + 50), 100, CV_RGB(0, 255, 0), -1, 8, 0);
    //cv::rectangle(mask, cv::Point(srcImage.cols/2-100, srcImage.rows/2-100), cv::Point(srcImage.cols/2+100, srcImage.rows/2+100), CV_RGB(255,0,0), -1, 8, 0);

    //description.push_back(std::make_pair(cv::Point(450 - 100, 940 - 100), cv::Point(450 + 100, 940 + 100)));
    description.push_back(std::make_pair(cv::Point(srcImage.rows/2-50, srcImage.cols/2-100), cv::Point(srcImage.rows/2+150, srcImage.cols/2+100)));

    auto iter = description.back();

    srcImage.copyTo(dstImage, mask);
    cv::imshow("image", dstImage);
    cv::waitKey(1);

    
    display_volumn(depth_map,mask, description);
    system("pause");
    zed.close();
}


