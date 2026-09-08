/*
 * Company:    AW
 * Author:     Penng
 * Date:    2026/01/15
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <chrono>

#include "model_config.h"
#include "yolo26_pipeline.h"

/* model_inputmeta.yml file param modify, eg:

    preproc_node_params:
      add_preproc_node: true
      preproc_type: IMAGE_RGB


demo model: model_rgb_xxx.nb.
*/

void get_input_data(const char* image_file, unsigned char* input_data, int letterbox_rows, int letterbox_cols)
{
    std::chrono::steady_clock::time_point Tbegin, Tend;
    Tbegin = std::chrono::steady_clock::now();

    cv::Mat sample = cv::imread(image_file, 1);
    if (sample.empty()) {
        fprintf(stderr, "cv::imread %s failed\n", image_file);
        return;
    }

    Tend = std::chrono::steady_clock::now();
    float f = std::chrono::duration_cast <std::chrono::milliseconds> (Tend - Tbegin).count();
//    std::cout << "preprocess cv::imread image file cost time : " << f << " ms" << std::endl;


    cv::Mat img;
    cv::cvtColor(sample, img, cv::COLOR_BGR2RGB);

    /* letterbox process to support different letterbox size */
    float scale_letterbox = 1.f;
    if ((letterbox_rows * 1.0 / img.rows) < (letterbox_cols * 1.0 / img.cols))
    {
        scale_letterbox = letterbox_rows * 1.0 / img.rows;
    }
    else
    {
        scale_letterbox = letterbox_cols * 1.0 / img.cols;
    }
    int resize_cols = int(round(scale_letterbox * img.cols));
    int resize_rows = int(round(scale_letterbox * img.rows));

    float dh = (float)(letterbox_rows - resize_rows);
    float dw = (float)(letterbox_cols - resize_cols);

    dh /= 2.0f;
    dw /= 2.0f;

    cv::resize(img, img, cv::Size(resize_cols, resize_rows));

    // create a mat with input_data ptr
    cv::Mat img_new(letterbox_rows, letterbox_cols, CV_8UC3, input_data);
    int top   = (int)(round(dh - 0.1));
    int bot   = (int)(round(dh + 0.1));
    int left  = (int)(round(dw - 0.1));
    int right = (int)(round(dw + 0.1));

    // Letterbox filling
    cv::copyMakeBorder(img, img_new, top, bot, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
}


int yolo26_preprocess(const char* imagepath, void* buff_ptr, unsigned int buff_size)
{
    int img_c = 3;

    // set default letterbox size
    int letterbox_rows = LETTERBOX_ROWS;
    int letterbox_cols = LETTERBOX_COLS;
    int img_size = letterbox_rows * letterbox_cols * img_c;

    unsigned int data_size = img_size * sizeof(uint8_t);

    if (data_size > buff_size) {
        printf("data size > buff size, please check code. \n");
        return -1;
    }

    get_input_data(imagepath, (unsigned char*)buff_ptr, letterbox_rows, letterbox_cols);

    return 0;
}

int yolo26_preprocess_frame(const cv::Mat& sample, void* buff_ptr,
                            unsigned int buff_size) {
    const unsigned int required = LETTERBOX_ROWS * LETTERBOX_COLS * 3;
    if (sample.empty() || buff_size < required) return -1;
    cv::Mat rgb;
    cv::cvtColor(sample, rgb, cv::COLOR_BGR2RGB);
    const float scale = std::min(static_cast<float>(LETTERBOX_ROWS) / rgb.rows,
                                 static_cast<float>(LETTERBOX_COLS) / rgb.cols);
    const int width = static_cast<int>(std::round(rgb.cols * scale));
    const int height = static_cast<int>(std::round(rgb.rows * scale));
    cv::resize(rgb, rgb, cv::Size(width, height));
    const int left = (LETTERBOX_COLS - width) / 2;
    const int right = LETTERBOX_COLS - width - left;
    const int top = (LETTERBOX_ROWS - height) / 2;
    const int bottom = LETTERBOX_ROWS - height - top;
    cv::Mat destination(LETTERBOX_ROWS, LETTERBOX_COLS, CV_8UC3, buff_ptr);
    cv::copyMakeBorder(rgb, destination, top, bottom, left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    return 0;
}



