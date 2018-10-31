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
      //! Gain
      int gainFactor;
    };

    class CaptureUeye: public Thread
    {
    public:
      //! Constructor.
      //! @param[in] task parent task.
      //! @param[in] buffer_capacity packet buffer capacity.
      CaptureUeye(DUNE::Tasks::Task* task, AOI aoi, HIDS cam = 1,
                  double fps = 30.0, unsigned pixel_clock = 1,
                  bool bin_v = false, bool bin_h = false):
        m_task(task),
        m_cam(cam),
        m_aoi(aoi),
        m_fps(fps),
        m_pixel_clock(pixel_clock),
        m_bin_v(bin_v),
        m_bin_h(bin_h),
        m_write(0),
        m_gain(0),
        m_lastTS(0)
      {
          
        m_height = aoi.height;
        m_width  = aoi.width;
        
        if (bin_v)
          m_height /= 2;
        if (bin_h)
          m_width /= 2;
        
        initializeCam();
      }

      //! Destructor.
      ~CaptureUeye(void)
      {
        is_ExitImageQueue(m_cam);
        is_ClearSequence(m_cam);

        // free buffers memory
        int i;
        for (i = (c_buf_len - 1); i >= 0; i--)
        {
          // free buffers
          if (is_FreeImageMem(m_cam, m_vpcSeqImgMem.at(i), m_viSeqMemId.at(i)))
          {
            m_task->err("FreeImageMem unsuccessful.");
          }
        }

        // no valid buffers any more
        m_viSeqMemId.clear();
        m_vpcSeqImgMem.clear();

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

        int tmp = is_AOI(m_cam, IS_AOI_IMAGE_SET_AOI, (void*) &rectAOI, sizeof (rectAOI));
        if (tmp)
          m_task->err("AOI unsuccessful. Error %d", tmp);
      }

      void
      allocateMemory(void)
      {
        int tmp;
        // Allocate memory
        for (unsigned i = 0; i < c_buf_len; i++)
        {
          INT iImgMemID = 0;
          char* pcImgMem = 0;

          // allocate a single buffer memory
          tmp = is_AllocImageMem(m_cam, m_width, m_height,
                                 16, &pcImgMem, &iImgMemID);

          if (tmp)
          {
            m_task->err("Allocate image buffer %d unsuccessful. Error %d", i, tmp);
            break;
          }

          // put memory into the sequence buffer management
          tmp = is_AddToSequence(m_cam, pcImgMem, iImgMemID);

          if (tmp)
          {
            // free latest buffer
            is_FreeImageMem(m_cam, pcImgMem, iImgMemID);
            m_task->err("AddToSequence %d unsuccessful. Error %d", i, tmp);
            break;
          }

          m_viSeqMemId.push_back(iImgMemID);
          m_vpcSeqImgMem.push_back(pcImgMem);
        }

        // enable the image queue
        tmp = is_InitImageQueue(m_cam, 0);
        if (tmp)
        {
          m_task->err("InitImageQueue unsuccessful. Error %d", tmp);
        }
      }

      void
      setPixelClock(unsigned pixel_clock)
      {
        UINT nNumberOfSupportedPixelClocks = 0;
        UINT nPixelClockList[150];

        INT nRet = is_PixelClock(m_cam, IS_PIXELCLOCK_CMD_GET_NUMBER,
                                 (void*) &nNumberOfSupportedPixelClocks,
                                 sizeof (nNumberOfSupportedPixelClocks));

        if ((nRet == IS_SUCCESS) && (nNumberOfSupportedPixelClocks > 0))
        {
          m_task->debug("Pixel Clock Number of values: %d.", nNumberOfSupportedPixelClocks);

          // No camera has more than 150 different pixel clocks.
          // Of course, the list can be allocated dynamically
          ZeroMemory(&nPixelClockList, sizeof (nPixelClockList));
          nRet = is_PixelClock(m_cam, IS_PIXELCLOCK_CMD_GET_LIST,
                               (void*) nPixelClockList,
                               nNumberOfSupportedPixelClocks * sizeof (UINT));

        }

        UINT nRange[3];
        ZeroMemory(nRange, sizeof (nRange));

        // Get pixel clock range
        nRet = is_PixelClock(m_cam, IS_PIXELCLOCK_CMD_GET_RANGE, (void*) nRange, sizeof (nRange));
        if (nRet == IS_SUCCESS)
        {
          m_task->debug("Pixel Clock range: [%d : %d : %d].", nRange[0], nRange[2], nRange[1]);
        }

        // Set the pixel clock. Higher pixel clock will enable higher FPS.
        int tmp = is_PixelClock(m_cam, IS_PIXELCLOCK_CMD_SET,
                                (void*) &nPixelClockList[pixel_clock],
                                sizeof (nPixelClockList[pixel_clock]));
        if (tmp != IS_SUCCESS)
          m_task->err("PixelClock unsuccessful. Error %d", tmp);
      }

      void
      setFPS(double fps)
      {
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
        is_Exposure(m_cam, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*) &newExp, 8);
        m_task->debug("Requested Exposure time %.2fms, actual is now %.2fms", exp, newExp);
      }

      void
      setGain(int gain)
      {
        m_gain = gain;

        //Disable auto gain control:
        double param = 0;
        int ret = is_SetAutoParameter(m_cam, IS_SET_ENABLE_AUTO_GAIN, &param, 0);
        if (ret == IS_SUCCESS)
          m_task->debug("Disabled Auto Gain.");
        else
          m_task->err("Disable Auto Gain unsuccessful. Error %d", ret);

        //Disable gain boost:
        ret = is_SetGainBoost(m_cam, IS_SET_GAINBOOST_OFF);
        if (ret == IS_SUCCESS)
          m_task->debug("Disabled Gain Boost.");
        else
          m_task->err("Disable Gain Boost unsuccessful. Error %d", ret);

        //Set HW gain to 0 (=1):
        ret = is_SetHardwareGain(m_cam, 0, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
        if (ret == IS_SUCCESS)
          m_task->debug("Disabled Digital Gain.");
        else
          m_task->err("SetHardwareGain unsuccessful. Error %d", ret);

        // Get sensor source gain range
        IS_RANGE_S32 rangeSourceGain;
        ret = is_DeviceFeature(m_cam, IS_DEVICE_FEATURE_CMD_GET_SENSOR_SOURCE_GAIN_RANGE,
                               (void*) &rangeSourceGain, sizeof (rangeSourceGain));

        if (ret == IS_SUCCESS)
        {
          m_task->debug("Source Gain range: [%d : %d : %d].",
                        rangeSourceGain.s32Min, rangeSourceGain.s32Inc, rangeSourceGain.s32Max);

          ret = is_DeviceFeature(m_cam, IS_DEVICE_FEATURE_CMD_SET_SENSOR_SOURCE_GAIN,
                                 (void*) &gain, sizeof (gain));
          if (ret == IS_SUCCESS)
            m_task->debug("Set Sensor Gain to %d.", gain);
          else
            m_task->err("Set Sensor Gain unsuccessful. Error %d", ret);
        }
      }
      
      void
      setBinning(bool bin_v, bool bin_h)
      {
        int ret = is_SetBinning(m_cam, IS_GET_SUPPORTED_BINNING);
        if (ret)
        {
          m_task->debug("On-sensor binning supported modes:");
          if (ret & IS_BINNING_2X_VERTICAL)
            m_task->debug("2X Vertical");
          if (ret & IS_BINNING_2X_HORIZONTAL)
            m_task->debug("2X Horizontal");
          if (ret & IS_BINNING_3X_VERTICAL)
            m_task->debug("3X Vertical");
          if (ret & IS_BINNING_3X_HORIZONTAL)
            m_task->debug("3X Horizontal");
          if (ret & IS_BINNING_4X_VERTICAL)
            m_task->debug("4X Vertical");
          if (ret & IS_BINNING_4X_HORIZONTAL)
            m_task->debug("4X Horizontal");
          if (ret & IS_BINNING_5X_VERTICAL)
            m_task->debug("5X Vertical");
          if (ret & IS_BINNING_5X_HORIZONTAL)
            m_task->debug("5X Horizontal");
          if (ret & IS_BINNING_6X_VERTICAL)
            m_task->debug("6X Vertical");
          if (ret & IS_BINNING_6X_HORIZONTAL)
            m_task->debug("6X Horizontal");
          if (ret & IS_BINNING_8X_VERTICAL)
            m_task->debug("8X Vertical");
          if (ret & IS_BINNING_8X_HORIZONTAL)
            m_task->debug("8X Horizontal");
          if (ret & IS_BINNING_16X_VERTICAL)
            m_task->debug("16X Vertical");
          if (ret & IS_BINNING_16X_HORIZONTAL)
            m_task->debug("16X Horizontal");
          
          int bin_factor_request = 0;
          
          if (bin_v)
            bin_factor_request |= IS_BINNING_2X_VERTICAL;
          if (bin_h)
            bin_factor_request |= IS_BINNING_2X_HORIZONTAL;
          
          if (bin_factor_request)
          {
            ret = is_SetBinning(m_cam, bin_factor_request);
            if (ret == IS_SUCCESS)
              m_task->inf("On-sensor binning successful.");
            else
              m_task->err("On-sensor binning unsuccessful. Error %d", ret);
          }

        }
        else
          m_task->war("On-sensor binning not supported.");
      }

      bool
      readFrame(Frame &frame_ret)
      {
        if (m_frame_buffer.empty())
          return false;
        else
        {
          frame_ret = m_frame_buffer.front();
          m_frame_buffer.pop();
          return true;
        }
      }

      void
      stopCapture(void)
      {
        int ret = is_StopLiveVideo(m_cam, IS_FORCE_VIDEO_STOP);

        if (ret == IS_SUCCESS)
          m_task->inf("Image capture stopped");
        else
          m_task->err("StopLiveVideo unsuccessful. Error %d", ret);

        UINT fMode = IO_FLASH_MODE_OFF;
        ret = is_IO(m_cam, IS_IO_CMD_FLASH_SET_MODE, (void*) &fMode, sizeof (fMode));
        if (ret == IS_SUCCESS)
          m_task->inf("Disabled Flash");
        else
          m_task->err("Disable Flash unsuccessful. Error %d", ret);
      }

      int
      queryGainFactor(int gain)
      {
        return is_SetHWGainFactor(m_cam, IS_INQUIRE_MASTER_GAIN_FACTOR, gain);
      }

    private:
      //! Parent task.
      DUNE::Tasks::Task* m_task;
      //! Queue of captured frames.
      std::queue<Frame> m_frame_buffer;
      //! Camera handle.
      HIDS m_cam;
      //! Will contain the address of memory allocated for images.
      char* m_ppcImgMem;
      //! Area of Interest.
      AOI m_aoi;
      //! Frames per Second.
      double m_fps;
      //! Pixel Clock.
      unsigned m_pixel_clock;
      //! Bin vertical
      bool m_bin_v;
      //! Bin horizontal
      bool m_bin_h;
      //! Will contain picture IDs for the above array.
      std::vector<INT> m_viSeqMemId;
      //! Will contain pointers to all image allocated memory areas.
      std::vector<char*> m_vpcSeqImgMem;
      //! Buffer Size
      static const unsigned c_buf_len = 32;
      //! Writer positions.
      unsigned m_write;
      //! Current gain factor.
      int m_gain;
      //! Last timestamp.
      unsigned long long m_lastTS;
      //! Frame height.
      unsigned m_height;
      //! Frame width.
      unsigned m_width;

      void
      initializeCam(void)
      {
        int tmp;
        // Will contain information about the sensor
        SENSORINFO sensorInfo;

        // Starts the driver and establishes the connection to the camera
        tmp = is_InitCamera(&m_cam, NULL);

        // Check if camera initialization was successful
        if (tmp != IS_SUCCESS)
          m_task->err("Camera initialization unsuccessful. Error %d", tmp);

        // Enables automatic closing of the camera handle after a camera has been removed on-the-fly
        tmp = is_EnableAutoExit(m_cam, IS_ENABLE_AUTO_EXIT);

        // Check if EnableAutoExit was successful
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

        // Set sensor bit depth
        UINT bitDepth = IS_SENSOR_BIT_DEPTH_12_BIT;
        tmp = is_DeviceFeature(m_cam, IS_DEVICE_FEATURE_CMD_SET_SENSOR_BIT_DEPTH, (void*) &bitDepth, sizeof (bitDepth));
        if (tmp != IS_SUCCESS)
        {
          m_task->war("DeviceFeature setting 12 bit unsuccessful. Error %d", tmp);
        }

        // Set color mode
        tmp = is_SetColorMode(m_cam, IS_CM_SENSOR_RAW12);
        if (tmp != IS_SUCCESS)
        {
          m_task->war("SetColorMode RAW12 unsuccessful. Error %d", tmp);
          
          tmp = is_SetColorMode(m_cam, IS_CM_SENSOR_RAW8);
          if (tmp != IS_SUCCESS)
            m_task->err("SetColorMode RAW8 unsuccessful. Error %d", tmp);
          else
            m_task->debug("SetColorMode RAW8 successful.");
        }

        // Enable Flash output for synchronization
        UINT fMode = IO_FLASH_MODE_FREERUN_HI_ACTIVE;
        tmp = is_IO(m_cam, IS_IO_CMD_FLASH_SET_MODE, (void*) &fMode, sizeof (fMode));
        if (tmp != IS_SUCCESS)
          m_task->err("Enable Flash unsuccessful. Error %d", tmp);

        fMode = IS_FLASH_AUTO_FREERUN_OFF;
        tmp = is_IO(m_cam, IS_IO_CMD_FLASH_SET_AUTO_FREERUN, (void*) &fMode, sizeof (fMode));
        if (tmp != IS_SUCCESS)
          m_task->err("Disable auto Flash unsuccessful. Error %d", tmp);

        // Set flash duration.
        IO_FLASH_PARAMS flashParams;
        flashParams.u32Duration = 10000;
        flashParams.s32Delay = 100;

        tmp = is_IO(m_cam, IS_IO_CMD_FLASH_SET_GPIO_PARAMS, (void*) &flashParams, sizeof (flashParams));
        if (tmp != IS_SUCCESS)
          m_task->err("Set Flash parameters unsuccessful. Error %d", tmp);

        setAOI(m_aoi);
        setBinning(m_bin_v,m_bin_h);
        setPixelClock(m_pixel_clock);
        setFPS(m_fps);
        is_SetDisplayMode(m_cam, IS_SET_DM_DIB);
        //        is_SetImageMem(m_cam, m_imgMems[0], m_imgMemIds[0]);
        allocateMemory();

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
        Time::Delay::wait(0.01);

        INT nMemID = 0;
        char* pBuffer = NULL;

        while (!isStopping() && !stat)
        {
          // run the the image queue acquisition
          stat = is_WaitForNextImage(m_cam, 1000, &pBuffer, &nMemID);
          if (stat)
          {
            if (stat != IS_TIMED_OUT)
              m_task->err("WaitEvent error: %d", stat);
          }
          else
          {
            m_task->spew("Captured frame!");

            Frame frame;

            UEYEIMAGEINFO imageInfo;
            int nRet = is_GetImageInfo(m_cam, nMemID, &imageInfo, sizeof (imageInfo));
            if (nRet == IS_SUCCESS)
            {
              // Get internal timestamp of image capture (tick count of the camera in 0.1 μs steps)
              m_task->spew("Time diff: %llu", imageInfo.u64TimestampDevice - m_lastTS);
              m_lastTS = imageInfo.u64TimestampDevice;

              // Get frame number
              frame.seqNum = imageInfo.u64FrameNumber;
              m_task->spew("Frame: %llu", frame.seqNum);
            }

            frame.data = (char*) std::malloc(m_height * m_width * 2);
            std::memcpy(frame.data, pBuffer, m_height * m_width * 2);
            frame.id = nMemID;
            frame.timestamp = Clock::getSinceEpoch();
            frame.gainFactor = m_gain;

            m_frame_buffer.push(frame);
            if (m_frame_buffer.size() > c_buf_len)
            {
              m_task->err("Buffer overrun!");
            }

            // do not forget to unlock the buffer, when all buffers are locked we cannot receive images any more
            is_UnlockSeqBuf(m_cam, nMemID, pBuffer);
          }
        }
      }
    };
  }
}

#endif
