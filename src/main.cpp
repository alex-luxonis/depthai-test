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

dai::Pipeline getUvcPipeline(std::string imageTuningPath) {
    dai::Pipeline pipeline;
    auto rgbCamera = pipeline.create<dai::node::ColorCamera>();
    rgbCamera->setBoardSocket(dai::CameraBoardSocket::RGB);
    rgbCamera->setInterleaved(false);
    rgbCamera->setPreviewSize(1920, 1080);
    rgbCamera->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    rgbCamera->initialControl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO);

    if (!imageTuningPath.empty()) {
        pipeline.setCameraTuningBlobPath(imageTuningPath);
    }

    rgbCamera->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    // scaling the output image size to 1080P
    rgbCamera->setIspScale(1, 2);

    // UVC
    auto uvc = pipeline.create<dai::node::UVC>();
    rgbCamera->video.link(uvc->input);

    // LED
    uvc->setGpiosOnInit({{58,0}, {37,0}, {34,0}});
    uvc->setGpiosOnStreamOn({{58,1}, {37,1}, {34,1}});
    uvc->setGpiosOnStreamOff({{58,0}, {37,0}, {34,0}});

    // Create an UAC (USB Audio Class) node
    auto uac = pipeline.create<dai::node::UAC>();
    uac->setStreamBackMic(false);
    uac->setMicGainDecibels(28);

    return pipeline;
}

std::vector<uint8_t> getDapPackage(std::string path) {
    std::ifstream file(path, std::ios::binary);
    if(!file.is_open()) throw std::runtime_error("Cannot open DAP file");

    return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
}

bool resetDevice() {
    // The reset is automatically done by the library when needed.
    // Also the default flash-booted PID is different now: 0xf63d
    return false;

    std::cout << "resetDevice()" << std::endl;
    libusb_device_handle *dev_handle;
    libusb_context *context = NULL;
    int r;

    r = libusb_init(NULL);
    if (r) {
        printf("libusb_init: %s\n", libusb_strerror((libusb_error)r));
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
        printf("libusb_control_transfer: %d %s\n", r, libusb_strerror((libusb_error)r));
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

bool flash(bool flashBootloader, bool flashBlConfig, bool flashPipeline, bool flashDap, std::string dapPath) {
    // Try resetting the device if already booted, and wait a bit for bootloader
    if (resetDevice() == true) this_thread::sleep_for(std::chrono::milliseconds(1000));

    bool found = false;
    dai::DeviceInfo info;
    std::tie(found, info) = dai::DeviceBootloader::getFirstAvailableDevice();
    if(!found) {
        std::cout << "No device found to flash. Exiting\n";
        return false;
    }
    std::cout << "Connecting to bootloader at MXID: " << info.getMxId() << "\n";
    dai::DeviceBootloader bl(info, true);

    //std::cout << std::setprecision(2) << std::fixed;

    // Create a progress callback lambda
    auto progress = [](float p) {
        std::cout << "Flashing progress... " << (p * 100) << "%\n";
    };

    auto tstart = steady_clock::now();
    bool success = false;
    std::string message;

    if (flashBootloader) {
        std::cout << "Flashing bootloader...\n";
        tstart = steady_clock::now();
        std::tie(success, message) = bl.flashBootloader(progress);
    }

    if (flashBlConfig) {
        std::cout << "Flashing bootloader config...\n";
        dai::DeviceBootloader::Config blcfg{};
        blcfg.usb.timeoutMs = 1000; // default 3000
        tstart = steady_clock::now();
        if(0) std::tie(success, message) = bl.flashConfigClear();
        else  std::tie(success, message) = bl.flashConfig(blcfg);
    }

    if (flashPipeline) {
        std::cout << "Flashing firmware and pipeline...\n";
        std::string tuningPath = ""; // TODO
        auto pipeline = getUvcPipeline(tuningPath);
        tstart = steady_clock::now();
        std::tie(success, message) = bl.flash(progress, pipeline);
    }

    if (flashDap) {
        std::cout << "Flashing DAP package...\n";
        auto package = getDapPackage(dapPath);
        tstart = steady_clock::now();
        std::tie(success, message) = bl.flashDepthaiApplicationPackage(progress, package);
    }

    if(success) {
        std::cout << "Flashing successful. Took "
                  << duration_cast<milliseconds>(steady_clock::now() - tstart).count()/1000.
                  << " seconds\n";
    } else {
        std::cout << "Flashing failed: " << message << "\n";
    }

    return success;
}

int main (int argc, char** argv) {
    using namespace std::chrono;

    bool flashBootloader = false;
    bool flashBlConfig = false;
    bool flashPipeline = false;
    bool flashDap = false;
    std::string dapPath = "";

    if(argc == 2) {
        auto option = std::string(argv[1]);
        if (option == "-fb") {
            flashBootloader = true;
        } else if (option == "-fbc") {
            flashBlConfig = true;
        } else if (option == "-f") {
            flashPipeline = true;
        } else if (option == "-h") {
            std::cout <<
                "Usage:\n"
                "    -fb     -- to flash bootloader\n"
                "    -fbc    -- to flash bootloader config\n"
                "    -f      -- to flash predefined UVC/UAC application/pipeline\n"
                "    <path>  -- to flash specified DAP file\n"
                "    no opt  -- to run main pipeline\n"
                ;
            return -1;
        } else {
            flashDap = true;
            dapPath = option;
        }
        return flash(flashBootloader, flashBlConfig, flashPipeline, flashDap, dapPath);
    }

    auto pipeline = getMainPipeline();
    while (true) {
        if (cameraInitialSetup()) {
            try {
                if (0) this_thread::sleep_for(std::chrono::milliseconds(2800));
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
