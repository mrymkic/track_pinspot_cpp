#include <windows.h>

#include <cmath>
#include <chrono>
#include <conio.h>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <cstring>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <array>

#include <k4a/k4a.h>
#include <k4abt.h>

namespace {

namespace fs = std::filesystem;

using Clock = std::chrono::steady_clock;

class ImageWindow {
public:
    bool create(const std::wstring &title, int width, int height)
    {
        width_ = width;
        height_ = height;

        WNDCLASSW wc{};
        wc.lpfnWndProc = &ImageWindow::wnd_proc;
        wc.hInstance = GetModuleHandleW(nullptr);
        wc.lpszClassName = L"TrackTestImageWindowClass";
        wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
        RegisterClassW(&wc);

        hwnd_ = CreateWindowExW(
            0,
            wc.lpszClassName,
            title.c_str(),
            WS_OVERLAPPEDWINDOW,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            width,
            height,
            nullptr,
            nullptr,
            wc.hInstance,
            nullptr);

        if (hwnd_ == nullptr) {
            return false;
        }

        ShowWindow(hwnd_, SW_SHOW);
        UpdateWindow(hwnd_);
        return true;
    }

    bool process_messages()
    {
        if (hwnd_ == nullptr) {
            return true;
        }

        MSG msg{};
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                return false;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        return IsWindow(hwnd_) != FALSE;
    }

    void show_bgra(const uint8_t *bgra_data)
    {
        if (hwnd_ == nullptr) {
            return;
        }

        BITMAPINFO bmi{};
        bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bmi.bmiHeader.biWidth = width_;
        bmi.bmiHeader.biHeight = -height_;
        bmi.bmiHeader.biPlanes = 1;
        bmi.bmiHeader.biBitCount = 32;
        bmi.bmiHeader.biCompression = BI_RGB;

        RECT client{};
        GetClientRect(hwnd_, &client);
        const int client_w = client.right - client.left;
        const int client_h = client.bottom - client.top;

        HDC hdc = GetDC(hwnd_);
        StretchDIBits(
            hdc,
            0,
            0,
            client_w,
            client_h,
            0,
            0,
            width_,
            height_,
            bgra_data,
            &bmi,
            DIB_RGB_COLORS,
            SRCCOPY);
        ReleaseDC(hwnd_, hdc);
    }

private:
    HWND hwnd_ = nullptr;
    int width_ = 0;
    int height_ = 0;

    static LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
    {
        if (msg == WM_DESTROY) {
            PostQuitMessage(0);
            return 0;
        }
        return DefWindowProc(hwnd, msg, wparam, lparam);
    }
};

struct PalConfig {
    std::string port;
    std::array<double, 3> vc;
    double rotate_deg;
};

struct AppConfig {
    PalConfig pal1;
    PalConfig pal2;
    PalConfig pal3;
    double update_interval_sec;
    double pt_factor;
};

std::array<double, 3> rotate_vc_y(const std::array<double, 3> &vc, double rotate_deg)
{
    constexpr double kPi = 3.14159265358979323846;
    const double angle = rotate_deg * kPi / 180.0;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {
        c * vc[0] + s * vc[2],
        vc[1],
        -s * vc[0] + c * vc[2]
    };
}

double parse_number(const std::string &text, const std::string &key)
{
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(text, m, pattern)) {
        throw std::runtime_error("Missing key: " + key);
    }
    return std::stod(m[1].str());
}

std::string parse_string(const std::string &text, const std::string &key)
{
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");
    std::smatch m;
    if (!std::regex_search(text, m, pattern)) {
        throw std::runtime_error("Missing key: " + key);
    }
    return m[1].str();
}

std::array<double, 3> parse_array3(const std::string &text, const std::string &key)
{
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\[([^\\]]+)\\]");
    std::smatch m;
    if (!std::regex_search(text, m, pattern)) {
        throw std::runtime_error("Missing key: " + key);
    }

    std::array<double, 3> values{};
    std::stringstream ss(m[1].str());
    std::string token;
    for (int i = 0; i < 3; ++i) {
        if (!std::getline(ss, token, ',')) {
            throw std::runtime_error("Invalid array size for key: " + key);
        }
        values[i] = std::stod(token);
    }
    return values;
}

std::string extract_object_block(const std::string &text, const std::string &name)
{
    const std::regex pattern("\\\"" + name + "\\\"\\s*:\\s*\\{([\\s\\S]*?)\\}");
    std::smatch m;
    if (!std::regex_search(text, m, pattern)) {
        throw std::runtime_error("Missing object: " + name);
    }
    return m[1].str();
}

PalConfig load_pal_config_from_text(const std::string &full_text, const std::string &pal_name)
{
    const std::string block = extract_object_block(full_text, pal_name);
    PalConfig cfg;
    cfg.port = parse_string(block, "port");
    cfg.rotate_deg = parse_number(block, "rotate");
    cfg.vc = rotate_vc_y(parse_array3(block, "vc"), cfg.rotate_deg);
    return cfg;
}

AppConfig load_config(const std::string &path)
{
    std::ifstream ifs(path);
    if (!ifs) {
        throw std::runtime_error("Failed to open config file: " + path);
    }
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string text = buffer.str();

    AppConfig cfg;
    cfg.pal1 = load_pal_config_from_text(text, "pal_1");
    cfg.pal2 = load_pal_config_from_text(text, "pal_2");
    cfg.pal3 = load_pal_config_from_text(text, "pal_3");
    cfg.update_interval_sec = parse_number(text, "update_interval");
    cfg.pt_factor = parse_number(text, "pt_factor");
    return cfg;
}

fs::path get_executable_path()
{
    std::vector<char> buffer(MAX_PATH, '\0');
    DWORD length = GetModuleFileNameA(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
    while (length == buffer.size()) {
        buffer.resize(buffer.size() * 2, '\0');
        length = GetModuleFileNameA(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
    }
    if (length == 0) {
        throw std::runtime_error("Failed to get executable path");
    }
    return fs::path(std::string(buffer.data(), length));
}

fs::path resolve_config_path(const std::string &raw_path)
{
    const fs::path input_path(raw_path);
    std::vector<fs::path> candidates;

    if (input_path.is_absolute()) {
        candidates.push_back(input_path);
    } else {
        candidates.push_back(fs::current_path() / input_path);

        const fs::path exe_dir = get_executable_path().parent_path();
        candidates.push_back(exe_dir / input_path);
        candidates.push_back(exe_dir.parent_path() / input_path);
        candidates.push_back(exe_dir.parent_path().parent_path() / input_path);
    }

    for (const auto &candidate : candidates) {
        if (fs::exists(candidate)) {
            return fs::canonical(candidate);
        }
    }

    std::ostringstream oss;
    oss << "Failed to open config file: " << raw_path << " (searched:";
    for (const auto &candidate : candidates) {
        oss << "\n  - " << candidate.string();
    }
    oss << "\n)";
    throw std::runtime_error(oss.str());
}

class PTUController {
public:
    explicit PTUController(const std::string &port_name)
    {
        const std::string full_port = "\\\\.\\" + port_name;
        handle_ = CreateFileA(
            full_port.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            nullptr);

        if (handle_ == INVALID_HANDLE_VALUE) {
            throw std::runtime_error("Failed to open serial port: " + port_name);
        }

        DCB dcb{};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(handle_, &dcb)) {
            throw std::runtime_error("GetCommState failed for port: " + port_name);
        }

        dcb.BaudRate = CBR_9600;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;

        if (!SetCommState(handle_, &dcb)) {
            throw std::runtime_error("SetCommState failed for port: " + port_name);
        }

        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout = 20;
        timeouts.ReadTotalTimeoutConstant = 100;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = 100;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        SetCommTimeouts(handle_, &timeouts);

        initialize();
    }

    ~PTUController()
    {
        close();
    }

    PTUController(const PTUController &) = delete;
    PTUController &operator=(const PTUController &) = delete;

    void move(int pan, int tilt)
    {
        send_command("PP" + std::to_string(pan));
        send_command("TP" + std::to_string(tilt));
        send_command("A");
    }

    void close()
    {
        if (handle_ != INVALID_HANDLE_VALUE) {
            send_command("FT");
            CloseHandle(handle_);
            handle_ = INVALID_HANDLE_VALUE;
        }
    }

private:
    HANDLE handle_ = INVALID_HANDLE_VALUE;
    std::mutex io_mutex_;

    void initialize()
    {
        send_command("FT");
        send_command("ED");
        send_command("PP");
        send_command("TP");
    }

    std::string send_command(const std::string &cmd)
    {
        std::lock_guard<std::mutex> lock(io_mutex_);

        const std::string full_cmd = cmd + "\r";
        DWORD written = 0;
        if (!WriteFile(handle_, full_cmd.data(), static_cast<DWORD>(full_cmd.size()), &written, nullptr)) {
            return "";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        std::string response;
        char ch = '\0';
        DWORD read = 0;
        for (;;) {
            if (!ReadFile(handle_, &ch, 1, &read, nullptr) || read == 0) {
                break;
            }
            if (ch == '\r' || ch == '\n') {
                if (!response.empty()) {
                    break;
                }
                continue;
            }
            response.push_back(ch);
        }
        return response;
    }
};

class CalcPanTilt {
public:
    explicit CalcPanTilt(const std::array<double, 3> &sensor_pos) : sensor_pos_(sensor_pos) {}

    double get_dir_hor(const std::array<double, 3> &joint_pos) const
    {
        const double dx = joint_pos[0] - sensor_pos_[0];
        const double dz = joint_pos[2] - sensor_pos_[2];
        return std::atan2(dx, dz);
    }

    double get_dir_ver(const std::array<double, 3> &joint_pos) const
    {
        const double dx = joint_pos[0] - sensor_pos_[0];
        const double dy = joint_pos[1] - sensor_pos_[1];
        const double dz = joint_pos[2] - sensor_pos_[2];
        const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (r <= 1e-9) {
            return 0.0;
        }
        return std::asin(dy / r);
    }

private:
    std::array<double, 3> sensor_pos_;
};

class TrackingController {
public:
    TrackingController(PTUController *ptu, std::array<double, 3> vc_pal, double update_interval_sec, double pt_factor)
        : ptu_(ptu),
          vc_pal_(std::move(vc_pal)),
          update_interval_sec_(update_interval_sec),
          pt_factor_(pt_factor),
          last_update_time_(Clock::now())
    {
    }

    void calculate_and_move(const std::array<double, 3> &left_ear_3d, Clock::time_point now, bool force_update)
    {
        CalcPanTilt calc(vc_pal_);

        const double dir_hor = calc.get_dir_hor(left_ear_3d);
        const double dir_ver = calc.get_dir_ver(left_ear_3d);

        const int p_con = -static_cast<int>(std::llround(dir_hor / 3.1416 * pt_factor_));
        const int t_con = 0;

        const double elapsed = std::chrono::duration<double>(now - last_update_time_).count();
        if (force_update || update_interval_sec_ <= 0.0 || elapsed >= update_interval_sec_) {
            ptu_->move(p_con, t_con);
            std::cout << "Horizontal direction: " << dir_hor
                      << ", Vertical direction: " << dir_ver << '\n';
            std::cout << "Pan angle: " << p_con << ", Tilt angle: " << t_con << '\n';
            last_update_time_ = now;
        }
    }

private:
    PTUController *ptu_;
    std::array<double, 3> vc_pal_;
    double update_interval_sec_;
    double pt_factor_;
    Clock::time_point last_update_time_;
};

void check_k4a(k4a_result_t result, const std::string &message)
{
    if (result != K4A_RESULT_SUCCEEDED) {
        throw std::runtime_error(message);
    }
}

void check_k4abt(k4a_wait_result_t result, const std::string &message)
{
    if (result != K4A_WAIT_RESULT_SUCCEEDED) {
        throw std::runtime_error(message);
    }
}

std::optional<std::array<float, 2>> convert_3d_to_color_2d(
    const k4a_calibration_t &calib,
    const std::array<double, 3> &p3d)
{
    k4a_float3_t src{};
    src.xyz.x = static_cast<float>(p3d[0]);
    src.xyz.y = static_cast<float>(p3d[1]);
    src.xyz.z = static_cast<float>(p3d[2]);

    k4a_float2_t dst{};
    int valid = 0;
    const k4a_result_t rc = k4a_calibration_3d_to_2d(
        &calib,
        &src,
        K4A_CALIBRATION_TYPE_DEPTH,
        K4A_CALIBRATION_TYPE_COLOR,
        &dst,
        &valid);
    if (rc != K4A_RESULT_SUCCEEDED || valid == 0) {
        return std::nullopt;
    }
    return std::array<float, 2>{dst.xy.x, dst.xy.y};
}

void draw_circle_bgra(
    std::vector<uint8_t> &image,
    int width,
    int height,
    int cx,
    int cy,
    int radius)
{
    const int r2 = radius * radius;
    for (int y = cy - radius; y <= cy + radius; ++y) {
        if (y < 0 || y >= height) {
            continue;
        }
        for (int x = cx - radius; x <= cx + radius; ++x) {
            if (x < 0 || x >= width) {
                continue;
            }
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy <= r2) {
                const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) * 4 + static_cast<size_t>(x) * 4;
                image[idx + 0] = 0;
                image[idx + 1] = 255;
                image[idx + 2] = 0;
                image[idx + 3] = 255;
            }
        }
    }
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        const std::string config_arg = (argc > 1) ? argv[1] : "track_config.json";
        const fs::path config_path = resolve_config_path(config_arg);
        std::cout << "Loading configuration...\n";
        std::cout << "Config path: " << config_path.string() << "\n";
        AppConfig config = load_config(config_path.string());

        std::cout << "Initializing PTU and Kinect...\n";
        PTUController ptu1(config.pal1.port);
        PTUController ptu2(config.pal2.port);
        PTUController ptu3(config.pal3.port);

        std::cout << "PTU : " << config.pal1.port << " " << config.pal2.port << " " << config.pal3.port << '\n';

        k4a_device_t device = nullptr;
        check_k4a(k4a_device_open(0, &device), "Failed to open Azure Kinect device");

        k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        device_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        device_config.synchronized_images_only = true;

        check_k4a(k4a_device_start_cameras(device, &device_config), "Failed to start cameras");

        k4a_calibration_t calibration{};
        check_k4a(
            k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &calibration),
            "Failed to get calibration");

        k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
        tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
        tracker_config.gpu_device_id = 0;
        tracker_config.model_path = "dnn_model_2_0_op11.onnx";

        k4abt_tracker_t tracker = nullptr;
        check_k4a(k4abt_tracker_create(&calibration, tracker_config, &tracker), "Failed to create body tracker");

        TrackingController pal1(&ptu1, config.pal1.vc, config.update_interval_sec, config.pt_factor);
        TrackingController pal2(&ptu2, config.pal2.vc, config.update_interval_sec, config.pt_factor);
        TrackingController pal3(&ptu3, config.pal3.vc, config.update_interval_sec, config.pt_factor);

        std::optional<std::array<double, 3>> last_left_ear;
        std::optional<std::array<float, 2>> last_left_ear_2d;
        ImageWindow image_window;
        bool window_initialized = false;

        std::cout << "Tracking started. Press 'u' to force update, 'q' to quit." << '\n';

        while (true) {
            if (window_initialized && !image_window.process_messages()) {
                break;
            }

            k4a_capture_t capture = nullptr;
            if (k4a_device_get_capture(device, &capture, 0) != K4A_WAIT_RESULT_SUCCEEDED) {
                continue;
            }

            k4a_image_t color_image = k4a_capture_get_color_image(capture);
            int color_width = 0;
            int color_height = 0;

            if (color_image != nullptr) {
                color_width = k4a_image_get_width_pixels(color_image);
                color_height = k4a_image_get_height_pixels(color_image);

                if (!window_initialized && color_width > 0 && color_height > 0) {
                    if (!image_window.create(L"Azure Kinect", color_width, color_height)) {
                        throw std::runtime_error("Failed to create image window");
                    }
                    window_initialized = true;
                }
            }

            if (k4abt_tracker_enqueue_capture(tracker, capture, 0) != K4A_WAIT_RESULT_SUCCEEDED) {
                if (color_image != nullptr) {
                    k4a_image_release(color_image);
                }
                k4a_capture_release(capture);
                continue;
            }

            k4abt_frame_t body_frame = nullptr;
            if (k4abt_tracker_pop_result(tracker, &body_frame, 0) != K4A_WAIT_RESULT_SUCCEEDED) {
                k4a_capture_release(capture);
                continue;
            }

            const size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
            auto now = Clock::now();

            for (size_t body_id = 0; body_id < num_bodies; ++body_id) {
                k4abt_skeleton_t skeleton{};
                if (k4abt_frame_get_body_skeleton(body_frame, static_cast<uint32_t>(body_id), &skeleton) != K4A_RESULT_SUCCEEDED) {
                    continue;
                }

                const auto &left = skeleton.joints[K4ABT_JOINT_EAR_LEFT].position;
                const auto &right = skeleton.joints[K4ABT_JOINT_EAR_RIGHT].position;

                std::array<double, 3> left_ear_3d{left.v[0], left.v[1], left.v[2]};
                std::array<double, 3> right_ear_3d{right.v[0], right.v[1], right.v[2]};

                auto update_pal = [&](TrackingController *pal) {
                    pal->calculate_and_move(left_ear_3d, now, false);
                };

                std::vector<std::thread> threads;
                threads.emplace_back(update_pal, &pal1);
                threads.emplace_back(update_pal, &pal2);
                threads.emplace_back(update_pal, &pal3);
                for (auto &th : threads) {
                    th.join();
                }

                last_left_ear = left_ear_3d;
                last_left_ear_2d = convert_3d_to_color_2d(calibration, left_ear_3d);
                (void)right_ear_3d;
            }

            if (color_image != nullptr && window_initialized && color_width > 0 && color_height > 0) {
                const uint8_t *src = k4a_image_get_buffer(color_image);
                const size_t image_size = static_cast<size_t>(color_width) * static_cast<size_t>(color_height) * 4;

                std::vector<uint8_t> display_buffer(image_size);
                std::memcpy(display_buffer.data(), src, image_size);

                if (num_bodies == 0 || !last_left_ear_2d.has_value()) {
                    draw_circle_bgra(display_buffer, color_width, color_height, 0, 0, 20);
                } else {
                    const int cx = static_cast<int>((*last_left_ear_2d)[0]);
                    const int cy = static_cast<int>((*last_left_ear_2d)[1]);
                    draw_circle_bgra(display_buffer, color_width, color_height, cx, cy, 20);
                }

                image_window.show_bgra(display_buffer.data());
            }

            k4abt_frame_release(body_frame);
            if (color_image != nullptr) {
                k4a_image_release(color_image);
            }
            k4a_capture_release(capture);

            if (_kbhit()) {
                const int key = _getch();
                if (key == 'q') {
                    ptu1.move(0, 0);
                    ptu2.move(0, 0);
                    ptu3.move(0, 0);
                    break;
                }
                if (key == 'u' && last_left_ear.has_value()) {
                    auto force_update = [&](TrackingController *pal) {
                        pal->calculate_and_move(*last_left_ear, Clock::now(), true);
                    };

                    std::vector<std::thread> threads;
                    threads.emplace_back(force_update, &pal1);
                    threads.emplace_back(force_update, &pal2);
                    threads.emplace_back(force_update, &pal3);
                    for (auto &th : threads) {
                        th.join();
                    }
                    std::cout << "Manually updated\n";
                }
            }
        }

        k4abt_tracker_shutdown(tracker);
        k4abt_tracker_destroy(tracker);
        k4a_device_stop_cameras(device);
        k4a_device_close(device);

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
