#include "depthai/depthai.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <iostream>
#include <memory>
#include <cstdio>
#include <libusb.h>


#define DEV_VID    0x03e7
#define DEV_PID    0xf63b

using namespace std::chrono;
using namespace std;

std::unique_ptr<dai::DeviceInfo> _currentDeviceInfo;
std::unique_ptr<dai::Device> _device;

dai::Pipeline getMainPipeline() {
    dai::Pipeline pipeline;
    // if (!_tuningFilePath.empty()) {
    //     pipeline.setCameraTuningBlobPath(_tuningFilePath);
    // }
    pipeline.setXLinkChunkSize(0);

    auto colorCamera = pipeline.create<dai::node::ColorCamera>();
    colorCamera->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCamera->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCamera->setInterleaved(false);
    colorCamera->setFps(30);

    colorCamera->initialControl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_VIDEO);
    colorCamera->initialControl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
    colorCamera->initialControl.setAutoExposureEnable();
    colorCamera->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);

    colorCamera->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    colorCamera->setIspScale(1, 2);

    colorCamera->setFp16(false);
    colorCamera->setPreviewSize(1920, 1080);

    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    xlinkOut->setFpsLimit(30);
    xlinkOut->setStreamName("rgb");
    xlinkOut->input.setBlocking(false);
    xlinkOut->input.setQueueSize(1);

    colorCamera->setVideoSize(1920, 1080);
    colorCamera->video.link(xlinkOut->input);

    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    controlIn->setStreamName("control");
    controlIn->out.link(colorCamera->inputControl);

    return pipeline;
}

bool resetDevice() {
    std::cout << "resetDevice()" << std::endl;
    libusb_device_handle *dev_handle;
    libusb_context *context = NULL;
    int r;

    r = libusb_init(NULL);
    if (r) {
        printf("libusb_init: %s\n", libusb_strerror(r));
        return false;
    }

    dev_handle = libusb_open_device_with_vid_pid(context, DEV_VID, DEV_PID);
    if (dev_handle == NULL) {
        printf("libusb_open_device_with_vid_pid: ERROR\n");
        return false;
    }

    r = libusb_control_transfer(dev_handle,
                                0x00,   // bmRequestType: device-directed
                                0xF5,   // bRequest: custom
                                0x0DA1, // wValue: custom
                                0x0000, // wIndex
                                NULL,   // data pointer
                                0,      // data size
                                1000    // timeout [ms]
    );
    if (r) {
        printf("libusb_control_transfer: %d %s\n", r, libusb_strerror(r));
    } else {
        printf("OK\n");
    }

    libusb_close(dev_handle);
    libusb_exit(context);
    return true;
}

bool cameraInitialSetup() {
    std::cout<<"cameraInitialSetup()"<<std::endl;
    bool found = false;
    dai::DeviceInfo deviceInfo = {};
    constexpr auto waitTime = std::chrono::milliseconds(500);
    std::tie(found, deviceInfo) = dai::Device::getAnyAvailableDevice(waitTime);

    if (found) {
        // If device is booted (in UVC mode), then triggering the reset
        if (deviceInfo.state == X_LINK_BOOTED) {
            resetDevice();
            return false;
        }
        _currentDeviceInfo = std::make_unique<dai::DeviceInfo>(deviceInfo);
        return true;
    }
    std::cout << "Camera_Not_Found" << std::endl;
    return false;
}

int main () {
    auto pipeline = getMainPipeline();
    while (true) {
        if (cameraInitialSetup()) {
            try {
                this_thread::sleep_for(std::chrono::milliseconds(2800));
                _device = std::make_unique<dai::Device>(dai::Pipeline().getOpenVINOVersion(), *(_currentDeviceInfo.get()));
                break;
            } catch (...) {
                std::cout << "something went wrong" << std::endl;
            }
        }
    }

    _device->startPipeline(pipeline);
    auto preview = _device->getOutputQueue("rgb");

    while (true) {
        try {
            auto imgFrame = preview->get<dai::ImgFrame>();
            std::cout << "frame - w: " << imgFrame->getWidth() << ", h: " << imgFrame->getHeight() << std::endl;
        } catch (const std::runtime_error& err) {
            std::cout << "error: "<< err.what() << std::endl;
            _device->close();
            break;
        }
    }
    return 0;
}