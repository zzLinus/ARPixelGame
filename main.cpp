#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/core/cvdef.h>
#define OLC_PGE_APPLICATION
#include <algorithm>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <stdint.h>
#include <vector>

// don't know why,but opencv need X11 related stuff include after it
#include "olcPixelGameEngine.h"

using namespace cv;
using namespace olc;
using namespace std;

class ARGame : public olc::PixelGameEngine {
  cv::VideoCapture cap;
  cv::Mat *frame = nullptr;
  cv::Mat *oldFrame = nullptr;
  cv::Mat *newFrame = nullptr;
  cv::Mat *frameGray = nullptr;
  cv::Mat *resizeFrameGray = nullptr;
  cv::Mat *resizeFrame = nullptr;
  float *fOldFrame = nullptr;
  float *fNewFrame = nullptr;
  float *fMotion = nullptr;
  float *vMatX = nullptr;
  float *vMatY = nullptr;
  bool initFlag = true;
  olc::Sprite *magenasiSpr = nullptr;
  olc::Decal *magenasiDec = nullptr;
  float magX = 0;
  float magY = 0;
  float magvX = 0;
  float magvY = 0;

public:
  ARGame() { sAppName = "ARGame"; }

public:
  bool OnUserCreate() override {
    cv::VideoCapture dcap(0);
    this->cap = dcap;
    this->frame = new cv::Mat;
    this->oldFrame = new cv::Mat;
    this->newFrame = new cv::Mat;
    this->frameGray = new cv::Mat;
    this->resizeFrame = new cv::Mat;
    this->resizeFrameGray = new cv::Mat;
    this->fOldFrame = new float[ScreenWidth() * ScreenHeight()];
    this->fNewFrame = new float[ScreenWidth() * ScreenHeight()];
    this->fMotion = new float[ScreenWidth() * ScreenHeight()];
    this->vMatX = new float[ScreenWidth() * ScreenHeight()];
    this->vMatY = new float[ScreenWidth() * ScreenHeight()];
    this->magenasiSpr = new olc::Sprite("./magenasi.png");
    this->magenasiDec = new olc::Decal(magenasiSpr);
    this->magX = 20;
    this->magY = 20;
    memset(vMatX, 0, sizeof(float) * ScreenWidth() * ScreenHeight());
    memset(vMatY, 0, sizeof(float) * ScreenWidth() * ScreenHeight());
    memset(fOldFrame, 0, sizeof(float) * ScreenWidth() * ScreenHeight());
    memset(fNewFrame, 0, sizeof(float) * ScreenWidth() * ScreenHeight());
    memset(fMotion, 0, sizeof(float) * ScreenWidth() * ScreenHeight());
    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {
    Clear(olc::BLACK);
    cap.read(*frame);
    cv::flip(*frame, *frame, 1);
    cv::cvtColor(*frame, *frameGray, cv::COLOR_BGR2GRAY);
    cv::resize(*frameGray, *resizeFrameGray,
               cv::Size(ScreenWidth(), ScreenHeight() / 2));
    cv::resize(*frame, *resizeFrame,
               cv::Size(ScreenWidth(), ScreenHeight() / 2));
    if (initFlag == true)
      this->resizeFrameGray->copyTo(*this->oldFrame);
    this->resizeFrameGray->copyTo(*this->newFrame);
    uint8_t B, G, R;
    uint8_t pixel = 0;
    float fDiff;
    float fOldPix, fNewPix;
    bool flag = true;

    // work out newf matrix oldf matrix motion matrix
    for (int y = 0; y < (ScreenHeight() / 2); y++)
      for (int x = 0; x < ScreenWidth(); x++) {
        if (this->initFlag == true)
          fOldFrame[y * ScreenWidth() + x] =
              (float)this->oldFrame->at<uint8_t>(y, x) / 255;
        fNewFrame[y * ScreenWidth() + x] =
            (float)this->newFrame->at<uint8_t>(y, x) / 255;
        fOldPix = fOldFrame[y * ScreenWidth() + x];
        fNewPix = fNewFrame[y * ScreenWidth() + x];
        fDiff = fNewPix - fOldPix;
        fOldFrame[y * ScreenWidth() + x] += fDiff * 0.2f;
        fDiff = std::fabs(fDiff);
        fMotion[y * ScreenWidth() + x] = (fDiff >= 0.1f) ? fDiff : 0.0f;
      }

    int patchSize = 9;
    int searchSize = 7;

    for (int y = 0; y < (ScreenHeight() / 2); y++) {
      for (int x = 0; x < ScreenWidth(); x++) {
        float fPatchDiffMax = INFINITY;
        for (int sy = 0; sy < searchSize; sy++)
          for (int sx = 0; sx < searchSize; sx++) {
            int searchX = x + (sx - searchSize / 2);
            int searchY = y + (sy - searchSize / 2);
            float diffSum = 0.0f;
            for (int py = 0; py < searchSize; py++)
              for (int px = 0; px < searchSize; px++) {
                int patchX = searchX + (px - patchSize / 2);
                int patchY = searchY + (py - patchSize / 2);
                int basePathX = x + (px - patchSize / 2);
                int basePathy = y + (py - patchSize / 2);
                float fBasePix = fOldFrame[patchY * ScreenWidth() + patchX];
                float fPatchPix = fNewFrame[py * ScreenWidth() + px];
                diffSum += std::fabs(fBasePix - fPatchPix);
              }

            if (diffSum < fPatchDiffMax) {
              fPatchDiffMax = diffSum;
              vMatX[y * ScreenWidth() + x] = (float)(searchX - x);
              vMatY[y * ScreenWidth() + x] = (float)(searchY - y);
            }
          }
      }
    }

    // use motion filter mat to filter volocity matrix reduce noise
    for (int j = 0; j < ScreenHeight() * ScreenWidth(); j++) {
      vMatX[j] *= fMotion[j] > 0 ? 1.0f : 0.0f;
      vMatY[j] *= fMotion[j] > 0 ? 1.0f : 0.0f;
    }

    // update sprite's velocity
    this->magvX += 0.3f * vMatX[(int)magY * ScreenWidth() + (int)magX];
    this->magvY += 0.3f * vMatY[(int)magY * ScreenWidth() + (int)magX];

    // sprite position is updated by velocity
    this->magX += 1.0f * this->magvX;
    this->magY += 1.0f * this->magvY;

    if (magX >= ScreenWidth())
      magX -= (float)ScreenWidth();
    if (magY >= (float)ScreenHeight() / 2)
      magY -= (float)ScreenHeight() / 2;
    if (magX < 0)
      magX += (float)ScreenWidth();
    if (magY < 0)
      magY += (float)ScreenHeight() / 2;
    this->magvX *= 0.85;
    this->magvY *= 0.85;

    // draw the RGB img
    for (int y = 0; y < (ScreenHeight() / 2); y++)
      for (int x = 0; x < ScreenWidth(); x++) {
        pixel = std::fabs(vMatY[y * ScreenWidth() + x] * 255);
        G = resizeFrame->at<cv::Vec3b>(y, x)[1];
        B = resizeFrame->at<cv::Vec3b>(y, x)[0];
        R = resizeFrame->at<cv::Vec3b>(y, x)[2];
        Draw(x, y, olc::Pixel(pixel, pixel, pixel));
        Draw(x, y + ScreenHeight() / 2, olc::Pixel(R, G, B));
      }

    olc::vf2d magenasiPos = {magX - 4, magY - 4 + (float)ScreenHeight() / 2};
    // draw the sprite
    DrawDecal(magenasiPos, magenasiDec, {0.1, 0.1});

    this->newFrame->copyTo(*this->oldFrame);
    this->initFlag = false;
    return true;
  }

  bool OnUserDestroy() override {
    delete this->frame;
    delete this->oldFrame;
    delete this->newFrame;
    delete this->frameGray;
    delete this->resizeFrame;
    delete this->resizeFrameGray;
    delete this->fOldFrame;
    delete this->fNewFrame;
    delete this->fMotion;
    delete this->vMatX;
    delete this->vMatY;
    delete this->magenasiDec;
    delete this->magenasiSpr;
    return true;
  }
};

int main(int argc, char *argv[]) {
  ARGame game;
  if (game.Construct(67, 120, 16, 16))
    game.Start();

  return 0;
}
