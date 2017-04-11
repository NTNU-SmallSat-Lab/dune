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

#ifndef VISION_UEYE_CAPUEYE_HPP_INCLUDED_
#define VISION_UEYE_CAPUEYE_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <queue>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Vendor headers.
#include <ueye.h>

using DUNE_NAMESPACES;

namespace Vision
{
  //! Device driver for uEye Cameras.
  //!
  //! @author João Fortuna
  namespace UEye
  {
    //! Area of Interest
    struct AOI
    {
      //! Upper left corner coordinates
      unsigned x, y;
      //! Size
      unsigned width, height;
    };

    //! Frame
    struct Frame
    {
      //! Data pointer
      char* data;
      //! Sequence number
      unsigned long long seqNum;
      //! Timestamp
      double timestamp;
      //! ID
      unsigned int id;
      //! Gain in percent of original value
      int gainFactor;
    };

    class CaptureUeye: public Thread
    {
    public:
      //! Constructor.
      //! @param[in] task parent task.
      //! @param[in] buffer_capacity packet buffer capacity.
      CaptureUeye(DUNE::Tasks::Task* task, AOI aoi, HIDS cam = 1, double fps = 30.0):
        m_task(task),
        m_cam(cam),
        m_aoi(aoi),
        m_fps(fps),
        m_read(0),
        m_write(0),
        m_lastTS(0)
      {
        m_imgMems = new char*[c_buf_len];
        m_imgMemIds = new int[c_buf_len];
        m_frames = new Frame[c_buf_len];

        initializeCam();
      }

      //! Destructor.
      ~CaptureUeye(void)
      {
        for (unsigned i = 0; i < c_buf_len; i++)
        {
          int tmp = is_FreeImageMem(m_cam, m_imgMems[i], m_imgMemIds[i]);

          if (tmp)
          {
            m_task->err("Trying to free image buffer %d. Error %d", i, tmp);
            break;
          }
        }
        Memory::clear(m_imgMems);
        Memory::clear(m_imgMemIds);
        Memory::clear(m_frames);

        is_ExitCamera(m_cam);
      }

      void
      setAOI(AOI aoi)
      {
        IS_RECT rectAOI;

        rectAOI.s32X      = aoi.x;
        rectAOI.s32Y      = aoi.y;
        rectAOI.s32Width  = aoi.width;
        rectAOI.s32Height = aoi.height;

        int tmp = is_AOI(m_cam, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI));
        if (tmp)
          m_task->err("AOI unsuccessful. Error %d", tmp);

        // Allocate memory

        // Will hold the picture ID of an image. I think this is an image's
        // index within a memory area allocated to images
        int pid;

        for (unsigned i = 0; i < c_buf_len; i++)
        {
          tmp = is_AllocImageMem(m_cam, m_aoi.width, m_aoi.height, 8, &m_ppcImgMem, &pid);

          // Store pointer and picture ID for this memory
          m_imgMems[i] = m_ppcImgMem;
          m_imgMemIds[i] = pid;

          if (tmp)
          {
            m_task->err("Allocate image buffer %d unsuccessful. Error %d", i, tmp);
            break;
          }
        }
      }

      void
      setFPS(double fps)
      {
        // Set the pixel clock. Higher pixel clock will enable higher FPS.
        UINT nPixelClockDefault = 140;
        int tmp = is_PixelClock(m_cam, IS_PIXELCLOCK_CMD_SET,
            (void*)&nPixelClockDefault,
            sizeof(nPixelClockDefault));
        if (tmp != IS_SUCCESS)
          m_task->err("PixelClock unsuccessful. Error %d", tmp);
        
        // Set target FPS
        double newFPS, fpsWish = fps;
        is_SetFrameRate(m_cam, fpsWish, &newFPS);
        m_task->debug("Requested %.2f FPS, actual FPS is now %.2f", fpsWish, newFPS);
      }

      void
      setExposure(float exp)
      {
        // Set target Exposure time
        double newExp = exp;
        is_Exposure (m_cam, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*) &newExp, 8);
        m_task->debug("Requested Exposure time %.2fms, actual is now %.2fms", exp, newExp);
      }

      void
      setGain(bool autogain, bool gainboost, int gain)
      {
        //Enable or disable auto gain control:
        double param = autogain ? 1 : 0;
        int ret = is_SetAutoParameter (m_cam, IS_SET_ENABLE_AUTO_GAIN, &param, 0);
        if (ret == IS_SUCCESS)
          m_task->debug("%s Auto Gain.", autogain ? "Enabled" : "Disabled");
        else
          m_task->err("%s Auto Gain unsuccessful. Error %d", autogain ? "Enable" : "Disable", ret);
        
        //Enable or disable gain boost:
        int param_int = gainboost ? IS_SET_GAINBOOST_ON : IS_SET_GAINBOOST_OFF;
        ret = is_SetGainBoost (m_cam, param_int);
        if (ret == IS_SUCCESS)
          m_task->debug("%s Gain Boost.", gainboost ? "Enabled" : "Disabled");
        else
          m_task->err("%s Gain Boost unsuccessful. Error %d", gainboost ? "Enable" : "Disable", ret);

        //Set gain if not auto:
        ret = is_SetHardwareGain (m_cam, gain, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
        if (ret == IS_SUCCESS)
          m_task->debug("Set Gain to %d.", gain);
        else
          m_task->err("SetHardwareGain unsuccessful. Error %d", ret);
      }

      Frame*
      readFrame(void)
      {
        if ((m_read % c_buf_len) == (m_write % c_buf_len))
          return NULL;

        return &m_frames[m_read++ % c_buf_len];
      }

      void
      stopCapture(void)
      {
        int ret = is_StopLiveVideo(m_cam, IS_FORCE_VIDEO_STOP);
        if (ret == IS_SUCCESS)
          m_task->inf("Image capture stopped");
        else
          m_task->err("StopLiveVideo unsuccessful. Error %d", ret);
      }

      int
      queryGainFactor(int gain)
      {
          return is_SetHWGainFactor(m_cam, IS_INQUIRE_MASTER_GAIN_FACTOR, gain);
      }

    private:
      //! Parent task.
      DUNE::Tasks::Task* m_task;
      //! Array of captured frames.
      Frame* m_frames;
      //! Camera handle.
      HIDS m_cam;
      //! Will contain the address of memory allocated for images.
      char* m_ppcImgMem;
      //! Area of Interest.
      AOI m_aoi;
      //! Frames per Second.
      double m_fps;
      //! Will contain pointers to all image allocated memory areas
      char **m_imgMems;
      //! Will contain picture IDs for the above array
      int *m_imgMemIds;
      //! Buffer Size
      static const unsigned c_buf_len = 256;
      //! Reader/Writer positions.
      unsigned m_read, m_write;
      //! Lookup table for gain factors
      int m_gains[101];
      //! Last timestamp
      unsigned long long m_lastTS;

      void
      initializeCam(void)
      {
        int tmp;
        // Will contain information about the sensor
        SENSORINFO sensorInfo;

        // Starts the driver and establishes the connection to the camera
        tmp = is_InitCamera(&m_cam, NULL);

        // Check if camera initialization was successfull.
        if (tmp != IS_SUCCESS)
          m_task->err("Camera initialization unsuccessful. Error %d", tmp);

        // Enables automatic closing of the camera handle after a camera has been removed on-the-fly
        tmp = is_EnableAutoExit(m_cam, IS_ENABLE_AUTO_EXIT);

        // Check if EnableAutoExit was successfull.
        if (tmp != IS_SUCCESS)
          m_task->err("EnableAutoExit unsuccessful. Error %d", tmp);

        // Get information about the sensor type used in the camera
        tmp = is_GetSensorInfo(m_cam, &sensorInfo);
        if (tmp == IS_SUCCESS)
        {
          m_task->debug("Sensor name: %s", sensorInfo.strSensorName);
          m_task->debug("Resolution: %d x %d", sensorInfo.nMaxWidth, sensorInfo.nMaxHeight);
        }
        else
          m_task->err("GetSensorInfo unsuccessful. Error %d", tmp);

        // Set color mode.
        is_SetColorMode(m_cam, IS_CM_SENSOR_RAW8);

        setAOI(m_aoi);
        setFPS(m_fps);
        is_SetDisplayMode(m_cam, IS_SET_DM_DIB);
        is_SetImageMem(m_cam, m_imgMems[0], m_imgMemIds[0]);

        for (int i = 0; i <= 100; i++)
        {
          m_gains[i] = queryGainFactor(i);
        }

        // Enable the FRAME event. Triggers when a frame is ready in memory.
        tmp = is_EnableEvent(m_cam, IS_SET_EVENT_FRAME);
        if (tmp)
          m_task->debug("Enabled FRAME event with status %d", tmp);
      }

      void
      run(void)
      {
        int stat = is_CaptureVideo(m_cam, IS_DONT_WAIT);
        if (stat)
          m_task->err("Failed to activate image capture with error %d", stat);
        else
          m_task->inf("Image capture started");

        // Small delay to let camera start
        Time::Delay::wait(1.0);

        while (!isStopping() && !stat)
        {
          stat = is_SetImageMem(m_cam, m_imgMems[m_write % c_buf_len], m_imgMemIds[m_write % c_buf_len]);
          if (stat)
          {
            m_task->err("SetImageMem Error: %d", stat);

            Time::Delay::wait(0.1);
            continue;
          }

          stat = is_WaitEvent(m_cam, IS_SET_EVENT_FRAME, 1000);
          if (stat)
          {
            if (stat != IS_TIMED_OUT)
              m_task->err("WaitEvent error: %d", stat);
          }
          else
          {
            m_task->spew("Captured! %d", m_write);

            Frame frame;

            UEYEIMAGEINFO imageInfo;
            int nRet = is_GetImageInfo(m_cam, m_imgMemIds[m_write % c_buf_len], &imageInfo, sizeof(imageInfo));
            if (nRet == IS_SUCCESS)
            {
              // Get internal timestamp of image capture (tick count of the camera in 0.1 μs steps)
              m_task->spew("Time diff: %llu", imageInfo.u64TimestampDevice - m_lastTS);
              m_lastTS = imageInfo.u64TimestampDevice;

              // Get frame number
              frame.seqNum = imageInfo.u64FrameNumber;
              m_task->spew("Frame: %llu", frame.seqNum);
            }

            int gain = is_SetHardwareGain(m_cam, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
            is_GetImageMem(m_cam, (void**)(&m_ppcImgMem));

            frame.data = m_imgMems[m_write % c_buf_len];
            frame.id = m_imgMemIds[m_write % c_buf_len];
            frame.timestamp = Clock::getSinceEpoch();
            frame.gainFactor = m_gains[gain];

            m_frames[m_write++ % c_buf_len] = frame;

            if (m_write == m_read)
            {
              m_task->err("Buffer overrun!");
              Time::Delay::wait(0.1);
            }
          }
        }
      }
    };
  }
}

#endif
