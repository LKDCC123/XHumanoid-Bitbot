#pragma once
#include "DataStreamClient.h"
#include "bitbot_kernel/utils/priority.h"

#include <iostream>
#include <stdint.h>
#include <variant>
#include <thread>
#include <string>

class ViconCapture
{
public:
  ViconCapture()
  {

  }

  ~ViconCapture()
  {
    this->Stop();
    if (capture_thread_.joinable())
      capture_thread_.join();
  }

  void Connect(std::string host_ip)
  {
    using namespace ViconDataStreamSDK::CPP;
    if (client_.Connect(host_ip).Result == Result::Success)
    {
      std::cout << "connect to " << host_ip << std::endl;
    }
    else
    {
      std::cout << "connect to " << host_ip << "failed" << std::endl;
      return;
    }

    client_.EnableSegmentData();
    client_.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

    capture_thread_ = std::thread(&ViconCapture::Capture, this);

  }

  struct Output
  {
    size_t frame_num;
    double latency;

    // std::chrono::time_point<std::chrono::high_resolution_clock> time;

    // std::array<double, 9> rotation_mat;
    std::array<double, 4> rotation_quat;
    // std::array<double, 3> rotation_euler;
    std::array<double, 3> translation;
  };

  Output Data()
  {
    std::lock_guard<std::mutex> lock(output_mutex_);

    return output_;
  }

  void Stop()
  {
    run_ = false;
  }
private:

  void Capture()
  {
    using namespace ViconDataStreamSDK::CPP;

    if (!setProcessHighPriority(20))
    {
      printf("Failed to set process scheduling policy\n");
    }

    Output op;

    while (run_)
    {
      while (client_.GetFrame().Result != Result::Success)
      {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
      }
      // op.time = std::chrono::high_resolution_clock::now();

      Output_GetFrameNumber _Output_GetFrameNumber = client_.GetFrameNumber();
      op.frame_num = _Output_GetFrameNumber.FrameNumber;

      Output_GetFrameRate Rate = client_.GetFrameRate();
      // ss << "," << Rate.FrameRateHz;

      double latency = client_.GetLatencyTotal().Total;
      op.latency = latency;

      // Count the number of subjects
      unsigned int SubjectCount = client_.GetSubjectCount().SubjectCount;

      // for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
      for (unsigned int SubjectIndex = 0; SubjectIndex < 1; ++SubjectIndex)
      {
        // Get the subject name
        std::string SubjectName = client_.GetSubjectName(SubjectIndex).SubjectName;

        // Count the number of segments
        unsigned int SegmentCount = client_.GetSegmentCount(SubjectName).SegmentCount;

        for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
        {
          // Get the segment name
          std::string SegmentName = client_.GetSegmentName(SubjectName, SegmentIndex).SegmentName;

          // Get the global segment translation
          Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation =
            client_.GetSegmentGlobalTranslation(SubjectName, SegmentName);
          op.translation[0] = _Output_GetSegmentGlobalTranslation.Translation[0];
          op.translation[1] = _Output_GetSegmentGlobalTranslation.Translation[1];
          op.translation[2] = _Output_GetSegmentGlobalTranslation.Translation[2];

          // Get the global segment rotation in quaternion co-ordinates
          Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion =
            client_.GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName);
          op.rotation_quat[0] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[0];
          op.rotation_quat[1] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[1];
          op.rotation_quat[2] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[2];
          op.rotation_quat[3] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[3];
        }
      }

      {
        std::lock_guard<std::mutex> lock(output_mutex_);
        output_ = op;
      }
    }

  }

  bool run_ = true;

  std::vector<std::string> objects_name_;

  ViconDataStreamSDK::CPP::Client client_;

  std::thread capture_thread_;

  std::mutex output_mutex_;

  Output output_;

};


