//***************************************************************************
// Copyright 2013-2017 Norwegian University of Science and Technology (NTNU)*
// Centre for Autonomous Marine Operations and Systems (AMOS)               *
// Department of Engineering Cybernetics (ITK)                              *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: João Fortuna                                                     *
//***************************************************************************

// ISO C++ 98 headers.
#include <queue>
#include <string>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include <Vision/UEye/CaptureUeye.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#if defined(DUNE_SYS_HAS_OPENCV2_IMGCODECS_HPP)
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#endif

using DUNE_NAMESPACES;

namespace Vision
{
  //! Device driver for uEye Cameras.
  //!
  //! @author João Fortuna
  namespace UEye
  {
    //! %Task arguments.

    struct Arguments
    {
      //! Camera ID
      unsigned cam_id;
      //! Frames Per Second.
      unsigned fps;
      //! Path to log directory
      std::string log_dir;
      //! Area of Interest specification
      AOI aoi;
      //! Auto Gain
      bool auto_gain;
      //! Gain boost
      bool gain_boost;
      //! Gain
      int gain;
      //! Exposure time
      float exposure;
      //! Calibration mode
      bool calib_mode;
      //! Calibration delta
      float calib_delta;
      //! Binning factor
      int binning;
    };

    //! Device driver task.

    struct Task:public DUNE::Tasks::Task
    {
      //! %Frame width. Unclear if 640 or 960
      static const unsigned c_width = 640;
      //! %Frame height. 480 is total, 250 is usable.
      static const unsigned c_height = 250;
      //! Configuration parameters.
      Arguments m_args;
      //! %Destination log folder.
      Path m_log_dir;
      //! Camera handle.
      HIDS m_cam;
      //! Thread for image capture.
      CaptureUeye* m_capture;
      //! Frame
      Frame* m_frame;
      //! Current calibration gain
      int m_calib_gain;
      //! Time of last calibration gain change
      double m_calib_time;
      //! OpenCV frame.
      cv::Mat m_image_cv;
      Task(const std::string& name, Tasks::Context& ctx):
      Tasks::Task(name, ctx),
      m_log_dir(ctx.dir_log),
      m_cam(1),
      m_capture(NULL),
      m_frame(NULL),
      m_calib_gain(0),
      m_calib_time(0.0)
      {
        // Retrieve configuration values.
        paramActive(Tasks::Parameter::SCOPE_GLOBAL,
                    Tasks::Parameter::VISIBILITY_USER);

        param("Camera ID", m_args.cam_id)
                .defaultValue("1")
                .minimumValue("1")
                .description("ID of camera to open");

        param("Frames Per Second", m_args.fps)
                .defaultValue("30")
                .minimumValue("0")
                .maximumValue("75")
                .description("Frames per second");

        param("AOI - X", m_args.aoi.x)
                .defaultValue("0")
                .minimumValue("0")
                .description("X coordinate of upper left corner of AOI");

        param("AOI - Y", m_args.aoi.y)
                .defaultValue("0")
                .minimumValue("0")
                .description("Y coordinate of upper left corner of AOI");

        param("AOI - Width", m_args.aoi.width)
                .defaultValue("640")
                .minimumValue("0")
                .description("Width of AOI");

        param("AOI - Height", m_args.aoi.height)
                .defaultValue("480")
                .minimumValue("0")
                .description("Height of AOI");

        param("Auto Gain", m_args.auto_gain)
                .defaultValue("false")
                .description("Enable Auto Gain");

        param("Gain Boost", m_args.gain_boost)
                .defaultValue("false")
                .description("Enable Gain Boost");

        param("Gain", m_args.gain)
                .defaultValue("50")
                .units(Units::Percentage)
                .minimumValue("0")
                .maximumValue("100")
                .description("Sensor Gain");

        param("Exposure", m_args.exposure)
                .defaultValue("4")
                .units(Units::Millisecond)
                .minimumValue("1")
                .maximumValue("20")
                .description("Exposure Time");

        param("Log Dir", m_args.log_dir)
                .defaultValue("")
                .description("Path to Log Directory");

        param("Calibration Mode", m_args.calib_mode)
                .defaultValue("false")
                .description("Enable calibration mode");

        param("Calibration Delta", m_args.calib_delta)
                .defaultValue("1.0")
                .units(Units::Second)
                .minimumValue("0.1")
                .maximumValue("10.0")
                .description("Time interval for each gain in calibration mode");

        param("Binning Factor", m_args.binning)
                .defaultValue("1")
                .minimumValue("1")
                .maximumValue("20")
                .description("Binning factor in the horizontal axis");

        bind<IMC::LoggingControl>(this);
      }

      //! Update internal parameters.
      void
      onUpdateParameters(void)
      {
        m_log_dir = m_args.log_dir;
        m_cam = m_args.cam_id;
      }

      //! Acquire resources and buffers.
      void
      onResourceAcquisition(void)
      {
        m_capture = new CaptureUeye(this, m_args.aoi, m_cam, m_args.fps);
        m_capture->setGain(m_args.auto_gain, m_args.gain_boost, m_args.gain);
        m_capture->setExposure(m_args.exposure);
      }

      //! Release allocated resources.
      void
      onResourceRelease(void)
      {
        if (m_capture != NULL)
        {
          delete m_capture;
          m_capture = NULL;
        }

        if (m_frame != NULL)
        {
          delete m_frame;
          m_frame = NULL;
        }
      }

      //! Initialize resources and start capturing frames.
      void
      onResourceInitialization(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
      }
      void
      consume(const IMC::LoggingControl* msg)
      {
        if ((msg->getDestination() != getSystemId()))
          return;

        if (msg->op == IMC::LoggingControl::COP_CURRENT_NAME || msg->op == IMC::LoggingControl::COP_REQUEST_START)
        {
          m_log_dir = m_args.log_dir / msg->name;
          m_log_dir.create();
        }
      }
      void
      onRequestActivation(void)
      {
        IMC::LoggingControl log_ctl;
        log_ctl.op = IMC::LoggingControl::COP_REQUEST_CURRENT_NAME;
        dispatch(log_ctl);
        activate();
      }
      void
      onActivation(void)
      {
        m_log_dir.create();
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        m_capture->start();
      }
      void
      onDeactivation(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
        stopCapture();
      }

      //! Saves the image.
      void
      saveImage(Frame* frame)
      {
        Path file = m_log_dir / String::str("%07llu_%0.4f_%04d_%d.png", frame->seqNum, frame->timestamp, frame->gainFactor, m_args.gain_boost ? 1 : 0);

        m_image_cv = cv::Mat(m_args.aoi.height, m_args.aoi.width, CV_16UC1);
        std::memcpy(m_image_cv.ptr(), frame->data, m_args.aoi.height * m_args.aoi.width * 2);

        cv::flip(m_image_cv, m_image_cv, 0);

        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(0);
        
        if (m_args.binning > 1)
        {
          cv::Mat image_cv_bin = cv::Mat(m_args.aoi.height, m_args.aoi.width/m_args.binning, CV_16UC1);
          m_image_cv *= 2^4;
          
          cv::resize(m_image_cv, image_cv_bin, image_cv_bin.size(), 0, 0, cv::INTER_LINEAR);
          cv::imwrite(file.c_str(), image_cv_bin, compression_params);
          return;
        }

        cv::imwrite(file.c_str(), m_image_cv, compression_params);
      }
      void
      stopCapture(void)
      {
        m_capture->stopCapture();

        bool qhasdata = true;
        int i = -1;

        debug("Emptying buffer.");
        while (qhasdata)
        {
          m_frame = m_capture->readFrame();
          if (m_frame == NULL)
            qhasdata = false;
          else
            saveImage(m_frame);
          i++;
        }

        debug("%d images in buffer when stopping.", i);

        if (m_capture->isRunning())
          m_capture->stopAndJoin();
      }
      void
      onMain(void)
      {
        while (!stopping())
        {
          consumeMessages();

          if (!isActive())
          {
            Time::Delay::wait(0.5);
            continue;
          }

          m_frame = m_capture->readFrame();
          if (m_frame == NULL)
            Time::Delay::wait(0.5);
          else
          {
            saveImage(m_frame);

            double now = Time::Clock::get();
            double delta = now - m_calib_time;

            if (m_args.calib_mode && (delta > m_args.calib_delta))
            {
              m_calib_gain++;
              if (m_calib_gain > 100)
                m_calib_gain = 0;

              m_capture->setGain(false, false, m_calib_gain);
              m_calib_time = now;
            }
          }
        }

        stopCapture();
      }
    };
  }
}

DUNE_TASK
