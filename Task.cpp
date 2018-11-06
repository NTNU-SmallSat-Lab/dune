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
      //! Gain
      int gain;
      //! Exposure time
      float exposure;
      //! Digital binning factor
      int binning_d;
      //! Analog vertical binning
      bool binning_av;
      //! Analog horizontal binning
      bool binning_ah;
      //! Pixel Clock index
      unsigned pixel_clock;
      //! Save raw images
      bool raw;
    };

    //! Device driver task.

    struct Task:public DUNE::Tasks::Task
    {
      //! Configuration parameters.
      Arguments m_args;
      //! %Destination log folder.
      Path m_log_dir;
      //! Camera handle.
      HIDS m_cam;
      //! Thread for image capture.
      CaptureUeye* m_capture;
      //! Frame
      Frame m_frame;
      //! OpenCV frame.
      cv::Mat m_image_cv;

      Task(const std::string& name, Tasks::Context& ctx):
      Tasks::Task(name, ctx),
      m_log_dir(ctx.dir_log),
      m_cam(1),
      m_capture(NULL)
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
                .minimumValue("0")
                .description("Width of AOI");

        param("AOI - Height", m_args.aoi.height)
                .minimumValue("0")
                .description("Height of AOI");

        param("Gain", m_args.gain)
                .defaultValue("0")
                .description("Sensor Gain");

        param("Exposure", m_args.exposure)
                .units(Units::Millisecond)
                .description("Exposure Time");

        param("Log Dir", m_args.log_dir)
                .defaultValue("")
                .description("Path to Log Directory");

        param("Digital Binning Factor", m_args.binning_d)
                .defaultValue("1")
                .minimumValue("1")
                .maximumValue("16")
                .description("Digital binning factor in the horizontal axis");

        param("On-sensor Bin Vertical", m_args.binning_av)
                .defaultValue("false")
                .description("On-sensor 2x binning in the vertical axis");

        param("On-sensor Bin Horizontal", m_args.binning_ah)
                .defaultValue("false")
                .description("On-sensor 2x binning in the horizontal axis");

        param("Pixel Clock", m_args.pixel_clock)
                .minimumValue("0")
                .description("Pixel Clock value in step number");

        param("Save Raw", m_args.raw)
                .defaultValue("false")
                .description("Save images in raw format");

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
        m_capture = new CaptureUeye(this, m_args.aoi, m_cam, m_args.fps,
                                    m_args.pixel_clock, m_args.binning_av,
                                    m_args.binning_ah);
        m_capture->setGain(m_args.gain);
        m_capture->setExposure(m_args.exposure);
        
        int height = m_args.aoi.height;
        int width  = m_args.aoi.width;
        
        if (m_args.binning_av)
          height /= 2;
        if (m_args.binning_ah)
          width /= 2;
        
        m_image_cv = cv::Mat(height, width, CV_16UC1);
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
        std::memcpy(m_image_cv.ptr(), frame->data, m_image_cv.total() * m_image_cv.elemSize());
        
        // do not forget to unlock the buffer, when all buffers are locked we cannot receive images any more
        is_UnlockSeqBuf(m_cam, frame->id, frame->data);
        //m_image_cv.data = (uchar*) frame->data; //TODO: look into cv::imdecode

        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(0);

        if (m_args.raw)
        {
          Path file = m_log_dir / String::str("%07llu_%0.4f_%03d.raw", frame->seqNum, frame->timestamp, frame->gainFactor);

          std::ofstream outfile(file.c_str(), std::ofstream::out | std::ofstream::binary);
          
          if (m_args.binning_d > 1)
          {
            cv::Mat image_cv_bin = binImage(m_image_cv, m_args.binning_d);
            outfile.write((char*) image_cv_bin.data, image_cv_bin.total() * image_cv_bin.elemSize());
            outfile.close();
            
            return;
          }
          
          outfile.write((char*) m_image_cv.data, m_image_cv.total() * m_image_cv.elemSize());
          outfile.close();
        }
        else
        {
          Path file = m_log_dir / String::str("%07llu_%0.4f_%03d.png", frame->seqNum, frame->timestamp, frame->gainFactor);

          if (m_args.binning_d > 1)
          {
            cv::Mat image_cv_bin = binImage(m_image_cv, m_args.binning_d);

            cv::imwrite(file.c_str(), image_cv_bin, compression_params);

            return;
          }

          cv::imwrite(file.c_str(), m_image_cv, compression_params);
        }
      }
      
      cv::Mat
      binImage(cv::Mat input, int binFactor)
      {
        cv::Mat output = cv::Mat(input.rows, input.cols/binFactor, CV_16UC1);
        
        for(int i = 0; i < output.cols; i++)
        {
          int startCol = i * binFactor;
          cv::Mat tmpCol = cv::Mat(input.rows, 1, CV_64FC1);
          cv::reduce(input.colRange(startCol, startCol + binFactor), tmpCol, 1, CV_REDUCE_SUM, CV_64FC1);
          
          tmpCol.convertTo(tmpCol, CV_16UC1);
          tmpCol.copyTo(output.col(i));
        }
        
        return output;
      }
      
      void
      stopCapture(void)
      {
        m_capture->stopCapture();

        int i = 0;

        debug("Emptying buffer.");
        while (m_capture->readFrame(m_frame))
        {
          saveImage(&m_frame);
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

          if (!m_capture->readFrame(m_frame))
            Time::Delay::wait(0.5);
          else
            saveImage(&m_frame);
        }

        stopCapture();
      }
    };
  }
}

DUNE_TASK
