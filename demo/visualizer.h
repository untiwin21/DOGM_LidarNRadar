#pragma once

#include "dogm/dogm.h"
#include <opencv2/opencv.hpp>
#include <string>

namespace dogm {

/**
 * @brief DOGM 객체의 현재 상태를 시각화하여 OpenCV Mat 객체로 반환합니다.
 * * @param dogm 시각화할 DOGM 객체.
 * @return cv::Mat 시각화된 이미지.
 */
cv::Mat visualizeDOGM(const DOGM& dogm);

/**
 * @brief 시각화 이미지에 추가 정보를 텍스트로 오버레이합니다.
 * * @param image 텍스트를 추가할 이미지.
 * @param frame_index 현재 프레임 번호.
 * @param update_time 업데이트에 소요된 시간 (ms).
 */
void addInfoText(cv::Mat& image, int frame_index, long long update_time);

} // namespace dogm
