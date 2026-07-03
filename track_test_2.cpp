#include <windows.h>

#include <dxgi.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <conio.h>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <cstring>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <ctime>
#include <vector>
#include <array>

#include <k4a/k4a.h>
#include <k4abt.h>

namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

class ImageWindow {
public:
    bool create(
        const std::wstring &title,
        int width,
        int height,
        const std::wstring &overlay_text = L"")
    {
        width_ = width;
        height_ = height;
        overlay_text_ = overlay_text.empty() ? title : overlay_text;

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
        if (!overlay_text_.empty()) {
            RECT text_rect{18, 18, client_w - 18, client_h - 18};
            SetBkMode(hdc, TRANSPARENT);
            SetTextColor(hdc, RGB(255, 255, 0));
            DrawTextW(hdc, overlay_text_.c_str(), -1, &text_rect, DT_LEFT | DT_TOP);
        }
        if (!status_text_.empty() && Clock::now() < status_until_) {
            RECT status_rect{18, 54, client_w - 18, client_h - 18};
            SetBkMode(hdc, TRANSPARENT);
            SetTextColor(hdc, RGB(0, 255, 128));
            DrawTextW(hdc, status_text_.c_str(), -1, &status_rect, DT_LEFT | DT_TOP | DT_WORDBREAK);
        }
        ReleaseDC(hwnd_, hdc);
    }

    void show_status(
        const std::wstring &message,
        std::chrono::milliseconds duration = std::chrono::milliseconds(2200))
    {
        status_text_ = message;
        status_until_ = Clock::now() + duration;
    }

private:
    HWND hwnd_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    std::wstring overlay_text_;
    std::wstring status_text_;
    Clock::time_point status_until_{};

    static LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
    {
        (void)wparam;
        (void)lparam;
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
    bool enable_ptu = true;
    bool enable_body_tracking = true;
    bool enable_aux_body_tracking = false;
    double max_aux_body_match_error_mm = 600.0;
    bool allow_aux_unsynced_fallback = false;
    int32_t capture_timeout_ms = 1000;
    int32_t body_tracking_timeout_ms = 0;
    std::string base_kinect_serial;
    std::string aux_kinect_serial;
    std::string body_tracking_mode = "gpu";
    int32_t body_tracking_gpu_device_id = 0;
    std::string body_tracking_model_path = "dnn_model_2_0_op11.onnx";
    std::string tracked_ear = "left";
    std::string tracked_person_camera = "base";
    std::string fusion_mode = "depth_only";
    std::string aux_view_mode = "depth";
    // Aux Kinect relative position [mm] in base Kinect coordinates.
    std::array<double, 3> aux_translation_mm{};
    std::array<std::array<double, 3>, 3> aux_rotation_matrix{{
        {{0.0, 0.0, -1.0}},
        {{0.0, 1.0,  0.0}},
        {{1.0, 0.0,  0.0}}
    }};
    // Legacy fallback for older config files without rotation_matrix.
    bool use_aux_x_as_base_z = true;
    uint32_t subordinate_delay_off_master_usec = 160;
    bool aux_synchronized_images_only = false;
    bool enable_image_capture = false;
    std::string capture_output_dir = "calibration_images";
    bool capture_save_aux_depth = false;
    bool enable_fusion_trace_csv = false;
    std::string fusion_trace_csv_path = "fusion_eval/fusion_trace.csv";
};

struct ImageCaptureSession {
    fs::path session_dir;
    fs::path base_dir;
    fs::path aux_dir;
    fs::path aux_depth_dir;
    bool save_aux_depth = false;
    uint64_t next_index = 1;
};

struct CaptureSaveResult {
    std::string stem;
    fs::path base_path;
    fs::path aux_path;
    std::optional<fs::path> aux_depth_path;
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

std::string parse_string_optional(
    const std::string &text,
    const std::string &key,
    const std::string &default_value)
{
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch m;
    if (!std::regex_search(text, m, pattern)) {
        return default_value;
    }
    return m[1].str();
}

std::string to_lower_ascii(std::string value)
{
    for (char &ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

std::string normalize_tracked_ear(std::string value)
{
    value = to_lower_ascii(value);
    if (value == "left" || value == "left_ear") {
        return "left";
    }
    if (value == "right" || value == "right_ear") {
        return "right";
    }
    throw std::runtime_error("Invalid tracked_ear. Use 'left' or 'right'.");
}

std::string normalize_body_tracking_mode(std::string value)
{
    value = to_lower_ascii(value);
    if (value == "gpu") {
        return "gpu";
    }
    if (value == "cpu") {
        return "cpu";
    }
    if (value == "cuda" || value == "gpu_cuda") {
        return "gpu_cuda";
    }
    if (value == "tensorrt" || value == "gpu_tensorrt") {
        return "gpu_tensorrt";
    }
    if (value == "directml" || value == "gpu_directml") {
        return "gpu_directml";
    }
    throw std::runtime_error(
        "Invalid body_tracking_mode. Use 'gpu', 'cpu', 'gpu_cuda', 'gpu_tensorrt', or 'gpu_directml'.");
}

std::string normalize_tracked_person_camera(std::string value)
{
    value = to_lower_ascii(value);
    if (value.empty() || value == "base" || value == "main" || value == "master") {
        return "base";
    }
    if (value == "aux" || value == "sub" || value == "subordinate") {
        return "aux";
    }
    throw std::runtime_error("Invalid tracked_person_camera. Use 'base' or 'aux'.");
}

std::string normalize_fusion_mode(std::string value)
{
    value = to_lower_ascii(value);
    if (value.empty() ||
        value == "depth_only" ||
        value == "depth" ||
        value == "fused" ||
        value == "fused_depth_only") {
        return "depth_only";
    }
    if (value == "full_3d" ||
        value == "full" ||
        value == "pointcloud" ||
        value == "point_cloud" ||
        value == "full_pointcloud" ||
        value == "full_point_cloud") {
        return "full_3d";
    }
    throw std::runtime_error("Invalid fusion_mode. Use 'depth_only' or 'full_3d'.");
}

std::string normalize_aux_view_mode(std::string value)
{
    value = to_lower_ascii(value);
    if (value.empty() ||
        value == "depth" ||
        value == "grayscale" ||
        value == "gray" ||
        value == "mono" ||
        value == "monochrome") {
        return "depth";
    }
    if (value == "color" ||
        value == "colour" ||
        value == "rgb" ||
        value == "bgra") {
        return "color";
    }
    throw std::runtime_error("Invalid aux_view_mode. Use 'depth' or 'color'.");
}

std::array<double, 3> parse_array3_values(const std::string &values_text, const std::string &key)
{
    std::array<double, 3> values{};
    std::stringstream ss(values_text);
    std::string token;
    for (int i = 0; i < 3; ++i) {
        if (!std::getline(ss, token, ',')) {
            throw std::runtime_error("Invalid array size for key: " + key);
        }
        values[i] = std::stod(token);
    }
    return values;
}

std::array<double, 3> parse_array3(const std::string &text, const std::string &key)
{
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\[([^\\]]+)\\]");
    std::smatch m;
    if (!std::regex_search(text, m, pattern)) {
        throw std::runtime_error("Missing key: " + key);
    }
    return parse_array3_values(m[1].str(), key);
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

bool parse_bool_optional(const std::string &text, const std::string &key, bool default_value)
{
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(true|false)");
    std::smatch m;
    if (!std::regex_search(text, m, pattern)) {
        return default_value;
    }
    return m[1].str() == "true";
}

double parse_number_optional(const std::string &text, const std::string &key, double default_value)
{
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(text, m, pattern)) {
        return default_value;
    }
    return std::stod(m[1].str());
}

std::array<std::array<double, 3>, 3> make_legacy_rotation_matrix(bool use_aux_x_as_base_z)
{
    if (use_aux_x_as_base_z) {
        return {{
            {{0.0, 0.0, -1.0}},
            {{0.0, 1.0,  0.0}},
            {{1.0, 0.0,  0.0}}
        }};
    }

    return {{
        {{0.0, 0.0,  1.0}},
        {{0.0, 1.0,  0.0}},
        {{-1.0, 0.0, 0.0}}
    }};
}

std::optional<std::array<std::array<double, 3>, 3>> parse_matrix3_optional(
    const std::string &text,
    const std::string &key)
{
    const std::regex pattern(
        "\\\"" + key + "\\\"\\s*:\\s*\\[\\s*\\[([^\\]]+)\\]\\s*,\\s*\\[([^\\]]+)\\]\\s*,\\s*\\[([^\\]]+)\\]\\s*\\]");
    std::smatch m;
    if (!std::regex_search(text, m, pattern)) {
        return std::nullopt;
    }

    return std::array<std::array<double, 3>, 3>{{
        parse_array3_values(m[1].str(), key + "[0]"),
        parse_array3_values(m[2].str(), key + "[1]"),
        parse_array3_values(m[3].str(), key + "[2]")
    }};
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
    cfg.enable_ptu = parse_bool_optional(text, "enable_ptu", true);
    cfg.enable_body_tracking = parse_bool_optional(text, "enable_body_tracking", true);
    cfg.enable_aux_body_tracking = parse_bool_optional(text, "enable_aux_body_tracking", false);
    cfg.max_aux_body_match_error_mm =
        parse_number_optional(text, "max_aux_body_match_error_mm", 600.0);
    if (cfg.max_aux_body_match_error_mm < 0.0) {
        throw std::runtime_error("max_aux_body_match_error_mm must be non-negative.");
    }
    cfg.allow_aux_unsynced_fallback = parse_bool_optional(text, "allow_aux_unsynced_fallback", false);
    cfg.capture_timeout_ms = static_cast<int32_t>(
        std::llround(parse_number_optional(text, "capture_timeout_ms", 1000.0)));
    cfg.body_tracking_timeout_ms = static_cast<int32_t>(
        std::llround(parse_number_optional(text, "body_tracking_timeout_ms", 0.0)));
    cfg.base_kinect_serial = parse_string_optional(text, "base_kinect_serial", "");
    cfg.aux_kinect_serial = parse_string_optional(text, "aux_kinect_serial", "");
    cfg.body_tracking_mode = normalize_body_tracking_mode(parse_string_optional(text, "body_tracking_mode", "gpu"));
    cfg.body_tracking_gpu_device_id = static_cast<int32_t>(
        std::llround(parse_number_optional(text, "body_tracking_gpu_device_id", 0.0)));
    cfg.body_tracking_model_path =
        parse_string_optional(text, "body_tracking_model_path", "dnn_model_2_0_op11.onnx");
    cfg.tracked_ear = normalize_tracked_ear(parse_string_optional(text, "tracked_ear", "left"));
    cfg.tracked_person_camera =
        normalize_tracked_person_camera(parse_string_optional(text, "tracked_person_camera", "base"));
    cfg.fusion_mode = normalize_fusion_mode(parse_string_optional(text, "fusion_mode", "depth_only"));
    cfg.aux_view_mode = normalize_aux_view_mode(parse_string_optional(text, "aux_view_mode", "depth"));

    cfg.aux_translation_mm = parse_array3(text, "aux_translation_mm");
    cfg.use_aux_x_as_base_z = parse_bool_optional(text, "use_aux_x_as_base_z", true);
    if (auto rotation_matrix = parse_matrix3_optional(text, "rotation_matrix")) {
        cfg.aux_rotation_matrix = *rotation_matrix;
    } else if (auto rotation_matrix = parse_matrix3_optional(text, "aux_rotation_matrix")) {
        cfg.aux_rotation_matrix = *rotation_matrix;
    } else {
        cfg.aux_rotation_matrix = make_legacy_rotation_matrix(cfg.use_aux_x_as_base_z);
    }
    cfg.subordinate_delay_off_master_usec = static_cast<uint32_t>(
        std::llround(parse_number_optional(text, "subordinate_delay_off_master_usec", 160.0)));
    cfg.aux_synchronized_images_only = parse_bool_optional(text, "aux_synchronized_images_only", false);
    cfg.enable_image_capture = parse_bool_optional(text, "enable_image_capture", false);
    cfg.capture_output_dir = parse_string_optional(text, "capture_output_dir", "calibration_images");
    cfg.capture_save_aux_depth = parse_bool_optional(text, "capture_save_aux_depth", false);
    cfg.enable_fusion_trace_csv = parse_bool_optional(text, "enable_fusion_trace_csv", false);
    cfg.fusion_trace_csv_path = parse_string_optional(text, "fusion_trace_csv_path", "fusion_eval/fusion_trace.csv");

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

fs::path resolve_asset_path(const std::string &raw_path, const fs::path &config_path)
{
    const fs::path input_path(raw_path);
    std::vector<fs::path> candidates;

    if (input_path.is_absolute()) {
        candidates.push_back(input_path);
    } else {
        candidates.push_back(fs::current_path() / input_path);
        candidates.push_back(config_path.parent_path() / input_path);
        candidates.push_back(get_executable_path().parent_path() / input_path);
    }

    for (const auto &candidate : candidates) {
        std::error_code ec;
        if (fs::exists(candidate, ec) && !ec) {
            return candidate;
        }
    }

    std::ostringstream oss;
    oss << "Failed to locate asset: " << raw_path << " (searched:";
    for (const auto &candidate : candidates) {
        oss << "\n  - " << candidate.string();
    }
    oss << "\n)";
    throw std::runtime_error(oss.str());
}

fs::path resolve_output_path(const std::string &raw_path, const fs::path &config_path)
{
    const fs::path input_path(raw_path);
    if (input_path.is_absolute()) {
        return input_path.lexically_normal();
    }
    return (config_path.parent_path() / input_path).lexically_normal();
}

std::string make_capture_session_name()
{
    const auto now = std::chrono::system_clock::now();
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    const std::time_t current_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_s(&local_time, &current_time);

    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y%m%d_%H%M%S")
        << '_' << std::setw(3) << std::setfill('0') << millis.count();
    return oss.str();
}

std::string make_capture_stem(uint64_t index)
{
    std::ostringstream oss;
    oss << std::setw(4) << std::setfill('0') << index;
    return oss.str();
}

void write_u16_le(std::ostream &os, uint16_t value)
{
    const char bytes[2] = {
        static_cast<char>(value & 0xffu),
        static_cast<char>((value >> 8) & 0xffu)
    };
    os.write(bytes, sizeof(bytes));
}

void write_u32_le(std::ostream &os, uint32_t value)
{
    const char bytes[4] = {
        static_cast<char>(value & 0xffu),
        static_cast<char>((value >> 8) & 0xffu),
        static_cast<char>((value >> 16) & 0xffu),
        static_cast<char>((value >> 24) & 0xffu)
    };
    os.write(bytes, sizeof(bytes));
}

void write_i32_le(std::ostream &os, int32_t value)
{
    write_u32_le(os, static_cast<uint32_t>(value));
}

void save_bgra32_image_as_bmp(const fs::path &path, k4a_image_t image)
{
    if (image == nullptr) {
        throw std::runtime_error("Color image is null.");
    }
    if (k4a_image_get_format(image) != K4A_IMAGE_FORMAT_COLOR_BGRA32) {
        throw std::runtime_error("Expected BGRA32 color image for BMP output.");
    }

    const int width = k4a_image_get_width_pixels(image);
    const int height = k4a_image_get_height_pixels(image);
    const int stride_bytes = k4a_image_get_stride_bytes(image);
    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Invalid color image dimensions.");
    }
    if (stride_bytes < width * 4) {
        throw std::runtime_error("Unexpected BGRA32 image stride.");
    }

    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {
        throw std::runtime_error("Failed to open output file: " + path.string());
    }

    const uint32_t pixel_data_size = static_cast<uint32_t>(width) * static_cast<uint32_t>(height) * 4u;
    const uint32_t file_size = 14u + 40u + pixel_data_size;

    write_u16_le(ofs, 0x4d42u);
    write_u32_le(ofs, file_size);
    write_u16_le(ofs, 0u);
    write_u16_le(ofs, 0u);
    write_u32_le(ofs, 54u);

    write_u32_le(ofs, 40u);
    write_i32_le(ofs, width);
    write_i32_le(ofs, -height);
    write_u16_le(ofs, 1u);
    write_u16_le(ofs, 32u);
    write_u32_le(ofs, 0u);
    write_u32_le(ofs, pixel_data_size);
    write_i32_le(ofs, 2835);
    write_i32_le(ofs, 2835);
    write_u32_le(ofs, 0u);
    write_u32_le(ofs, 0u);

    const uint8_t *buffer = k4a_image_get_buffer(image);
    for (int y = 0; y < height; ++y) {
        const uint8_t *row = buffer + static_cast<size_t>(y) * static_cast<size_t>(stride_bytes);
        ofs.write(reinterpret_cast<const char *>(row), static_cast<std::streamsize>(width * 4));
    }

    if (!ofs) {
        throw std::runtime_error("Failed while writing BMP file: " + path.string());
    }
}

void save_mjpg_image_as_jpeg(const fs::path &path, k4a_image_t image)
{
    if (image == nullptr) {
        throw std::runtime_error("Color image is null.");
    }
    if (k4a_image_get_format(image) != K4A_IMAGE_FORMAT_COLOR_MJPG) {
        throw std::runtime_error("Expected MJPG color image for JPEG output.");
    }

    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {
        throw std::runtime_error("Failed to open output file: " + path.string());
    }

    const uint8_t *buffer = k4a_image_get_buffer(image);
    const size_t size_bytes = k4a_image_get_size(image);
    ofs.write(reinterpret_cast<const char *>(buffer), static_cast<std::streamsize>(size_bytes));
    if (!ofs) {
        throw std::runtime_error("Failed while writing JPEG file: " + path.string());
    }
}

void save_depth16_image_as_pgm(const fs::path &path, k4a_image_t image)
{
    if (image == nullptr) {
        throw std::runtime_error("Depth image is null.");
    }
    if (k4a_image_get_format(image) != K4A_IMAGE_FORMAT_DEPTH16) {
        throw std::runtime_error("Expected DEPTH16 image for PGM output.");
    }

    const int width = k4a_image_get_width_pixels(image);
    const int height = k4a_image_get_height_pixels(image);
    const int stride_bytes = k4a_image_get_stride_bytes(image);
    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Invalid depth image dimensions.");
    }
    if (stride_bytes < width * 2) {
        throw std::runtime_error("Unexpected DEPTH16 image stride.");
    }

    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {
        throw std::runtime_error("Failed to open output file: " + path.string());
    }

    ofs << "P5\n" << width << ' ' << height << "\n65535\n";
    const uint8_t *buffer = k4a_image_get_buffer(image);
    for (int y = 0; y < height; ++y) {
        const uint8_t *row = buffer + static_cast<size_t>(y) * static_cast<size_t>(stride_bytes);
        for (int x = 0; x < width; ++x) {
            const uint16_t value =
                static_cast<uint16_t>(row[2 * x]) |
                (static_cast<uint16_t>(row[2 * x + 1]) << 8);
            const char bytes[2] = {
                static_cast<char>((value >> 8) & 0xffu),
                static_cast<char>(value & 0xffu)
            };
            ofs.write(bytes, sizeof(bytes));
        }
    }

    if (!ofs) {
        throw std::runtime_error("Failed while writing PGM file: " + path.string());
    }
}

fs::path save_kinect_color_image(const fs::path &directory, const std::string &stem, k4a_image_t image)
{
    const auto format = k4a_image_get_format(image);
    if (format == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
        const fs::path path = directory / (stem + ".bmp");
        save_bgra32_image_as_bmp(path, image);
        return path;
    }
    if (format == K4A_IMAGE_FORMAT_COLOR_MJPG) {
        const fs::path path = directory / (stem + ".jpg");
        save_mjpg_image_as_jpeg(path, image);
        return path;
    }
    throw std::runtime_error("Unsupported Kinect color image format for capture output.");
}

ImageCaptureSession create_image_capture_session(
    const AppConfig &config,
    const fs::path &config_path)
{
    ImageCaptureSession session;
    const fs::path output_root = resolve_output_path(config.capture_output_dir, config_path);
    session.session_dir = output_root / make_capture_session_name();
    session.base_dir = session.session_dir / "base";
    // `aux` is a reserved DOS device name on Windows, so use `aux_color`.
    session.aux_dir = session.session_dir / "aux_color";
    session.aux_depth_dir = session.session_dir / "aux_depth";
    session.save_aux_depth = config.capture_save_aux_depth;

    std::error_code ec;
    fs::create_directories(session.base_dir, ec);
    if (ec) {
        throw std::runtime_error("Failed to create base capture directory: " + session.base_dir.string());
    }
    fs::create_directories(session.aux_dir, ec);
    if (ec) {
        throw std::runtime_error("Failed to create aux capture directory: " + session.aux_dir.string());
    }
    if (session.save_aux_depth) {
        fs::create_directories(session.aux_depth_dir, ec);
        if (ec) {
            throw std::runtime_error("Failed to create aux depth capture directory: " + session.aux_depth_dir.string());
        }
    }

    return session;
}

CaptureSaveResult save_capture_pair(
    ImageCaptureSession &session,
    k4a_image_t base_color_image,
    k4a_image_t aux_color_image,
    k4a_image_t aux_depth_image)
{
    if (base_color_image == nullptr) {
        throw std::runtime_error("Base color image is not available for capture.");
    }
    if (aux_color_image == nullptr) {
        throw std::runtime_error("Aux color image is not available for capture.");
    }

    CaptureSaveResult result;
    result.stem = make_capture_stem(session.next_index++);
    result.base_path = save_kinect_color_image(session.base_dir, result.stem, base_color_image);
    result.aux_path = save_kinect_color_image(session.aux_dir, result.stem, aux_color_image);

    if (session.save_aux_depth && aux_depth_image != nullptr) {
        const fs::path depth_path = session.aux_depth_dir / (result.stem + ".pgm");
        save_depth16_image_as_pgm(depth_path, aux_depth_image);
        result.aux_depth_path = depth_path;
    }

    return result;
}

std::string format_capture_save_log_message(const CaptureSaveResult &result)
{
    std::ostringstream oss;
    oss << "Saved Kinect capture pair stem=" << result.stem
        << " base=" << result.base_path.string()
        << " aux=" << result.aux_path.string();
    if (result.aux_depth_path.has_value()) {
        oss << " aux_depth=" << result.aux_depth_path->string();
    }
    return oss.str();
}

std::wstring make_capture_status_text(const CaptureSaveResult &result)
{
    std::wstring text = L"Saved ";
    text += result.base_path.filename().wstring();
    text += L" / ";
    text += result.aux_path.filename().wstring();
    if (result.aux_depth_path.has_value()) {
        text += L" / ";
        text += result.aux_depth_path->filename().wstring();
    }
    return text;
}

fs::path get_runtime_log_path()
{
    try {
        return get_executable_path().parent_path() / "track_test_2_runtime.log";
    } catch (...) {
        return fs::current_path() / "track_test_2_runtime.log";
    }
}

void reset_runtime_log()
{
    std::ofstream ofs(get_runtime_log_path(), std::ios::trunc);
    (void)ofs;
}

void append_runtime_log(const std::string &message)
{
    std::ofstream ofs(get_runtime_log_path(), std::ios::app);
    if (ofs) {
        ofs << message << '\n';
    }
}

void info_and_log(const std::string &message)
{
    std::cout << message << '\n';
    append_runtime_log(message);
}

void warn_and_log(const std::string &message)
{
    std::cerr << "Warning: " << message << '\n';
    append_runtime_log("Warning: " + message);
}

std::array<int, 2> color_resolution_to_size(k4a_color_resolution_t color_resolution)
{
    switch (color_resolution) {
    case K4A_COLOR_RESOLUTION_720P:
        return {1280, 720};
    case K4A_COLOR_RESOLUTION_1080P:
        return {1920, 1080};
    case K4A_COLOR_RESOLUTION_1440P:
        return {2560, 1440};
    case K4A_COLOR_RESOLUTION_1536P:
        return {2048, 1536};
    case K4A_COLOR_RESOLUTION_2160P:
        return {3840, 2160};
    case K4A_COLOR_RESOLUTION_3072P:
        return {4096, 3072};
    default:
        return {1280, 720};
    }
}

std::array<int, 2> depth_mode_to_size(k4a_depth_mode_t depth_mode)
{
    switch (depth_mode) {
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
        return {320, 288};
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
        return {640, 576};
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
        return {512, 512};
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
        return {1024, 1024};
    case K4A_DEPTH_MODE_PASSIVE_IR:
        return {1024, 1024};
    default:
        return {512, 512};
    }
}

std::vector<uint8_t> make_depth_bgra_buffer(k4a_image_t depth_image)
{
    if (depth_image == nullptr) {
        return {};
    }

    const int width = k4a_image_get_width_pixels(depth_image);
    const int height = k4a_image_get_height_pixels(depth_image);
    const int stride_pixels = k4a_image_get_stride_bytes(depth_image) / static_cast<int>(sizeof(uint16_t));
    const auto *depth_buffer = reinterpret_cast<const uint16_t *>(k4a_image_get_buffer(depth_image));
    if (width <= 0 || height <= 0 || stride_pixels <= 0 || depth_buffer == nullptr) {
        return {};
    }

    std::vector<uint8_t> display_buffer(
        static_cast<size_t>(width) * static_cast<size_t>(height) * 4,
        0);

    constexpr uint16_t kDisplayMaxDepthMm = 5000;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const uint16_t depth_mm =
                depth_buffer[static_cast<size_t>(y) * static_cast<size_t>(stride_pixels) + static_cast<size_t>(x)];
            uint8_t gray = 0;
            if (depth_mm > 0) {
                const uint16_t clamped_depth =
                    (depth_mm < kDisplayMaxDepthMm) ? depth_mm : kDisplayMaxDepthMm;
                gray = static_cast<uint8_t>(
                    (static_cast<uint32_t>(kDisplayMaxDepthMm - clamped_depth) * 255u) /
                    kDisplayMaxDepthMm);
            }

            const size_t idx =
                static_cast<size_t>(y) * static_cast<size_t>(width) * 4 + static_cast<size_t>(x) * 4;
            display_buffer[idx + 0] = gray;
            display_buffer[idx + 1] = gray;
            display_buffer[idx + 2] = gray;
            display_buffer[idx + 3] = 255;
        }
    }

    return display_buffer;
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
            std::cout << "Horizontal direction: " << dir_hor
                      << ", Vertical direction: " << dir_ver << '\n';
            std::cout << "Pan angle: " << p_con << ", Tilt angle: " << t_con << '\n';
            if (ptu_ != nullptr) {
                ptu_->move(p_con, t_con);
            }
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

struct AuxKinectTransform {
    std::array<double, 3> translation_mm;
    std::array<std::array<double, 3>, 3> rotation_matrix;
};

struct JointSample3D {
    std::array<double, 3> position;
    k4abt_joint_confidence_level_t confidence_level = K4ABT_JOINT_CONFIDENCE_NONE;
    size_t body_index = 0;
};

struct EarCandidateInBase {
    JointSample3D sample;
    std::array<double, 3> position_in_base;
};

struct AuxDepthSample {
    std::array<double, 3> predicted_aux_point_3d;
    std::array<double, 3> sampled_aux_point_3d;
    std::array<float, 2> predicted_depth_2d;
    std::array<float, 2> sampled_depth_2d;
    double spatial_error_mm = std::numeric_limits<double>::infinity();
};

struct AuxDepthDebugInfo {
    std::string failure_reason = "not_attempted";
    std::optional<std::array<double, 3>> predicted_aux_point_3d;
    std::optional<std::array<float, 2>> predicted_depth_2d;
    int depth_width = 0;
    int depth_height = 0;
    int center_x = -1;
    int center_y = -1;
    int nonzero_depth_candidates = 0;
    int valid_3d_candidates = 0;
    double best_spatial_error_mm = std::numeric_limits<double>::infinity();
};

struct KinectDeviceIdentity {
    uint32_t index = 0;
    std::string serial;
};

void check_k4a(k4a_result_t result, const std::string &message)
{
    if (result != K4A_RESULT_SUCCEEDED) {
        throw std::runtime_error(message);
    }
}

std::string get_kinect_serial_number(k4a_device_t device)
{
    size_t serial_size = 64;
    std::vector<char> buffer(serial_size, '\0');
    k4a_buffer_result_t result = k4a_device_get_serialnum(device, buffer.data(), &serial_size);
    if (result == K4A_BUFFER_RESULT_TOO_SMALL) {
        buffer.assign(serial_size, '\0');
        result = k4a_device_get_serialnum(device, buffer.data(), &serial_size);
    }
    if (result != K4A_BUFFER_RESULT_SUCCEEDED) {
        throw std::runtime_error("Failed to read Azure Kinect serial number.");
    }
    return std::string(buffer.data());
}

std::string narrow_from_wide(const wchar_t *text)
{
    if (text == nullptr) {
        return {};
    }

    const int required_size = WideCharToMultiByte(CP_UTF8, 0, text, -1, nullptr, 0, nullptr, nullptr);
    if (required_size <= 1) {
        return {};
    }

    std::string utf8(static_cast<size_t>(required_size - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, text, -1, utf8.data(), required_size, nullptr, nullptr);
    return utf8;
}

std::string format_win32_error_message(DWORD error_code)
{
    LPWSTR buffer = nullptr;
    const DWORD length = FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,
        error_code,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        reinterpret_cast<LPWSTR>(&buffer),
        0,
        nullptr);

    std::string message;
    if (length > 0 && buffer != nullptr) {
        message = narrow_from_wide(buffer);
        while (!message.empty() && (message.back() == '\r' || message.back() == '\n' || message.back() == ' ')) {
            message.pop_back();
        }
        LocalFree(buffer);
    } else {
        message = "Unknown error";
    }
    return message;
}

std::optional<fs::path> search_dll_in_loader_path(const std::wstring &dll_name)
{
    DWORD required = SearchPathW(nullptr, dll_name.c_str(), nullptr, 0, nullptr, nullptr);
    if (required == 0) {
        return std::nullopt;
    }

    std::vector<wchar_t> buffer(static_cast<size_t>(required), L'\0');
    wchar_t *file_part = nullptr;
    required = SearchPathW(
        nullptr,
        dll_name.c_str(),
        nullptr,
        static_cast<DWORD>(buffer.size()),
        buffer.data(),
        &file_part);
    if (required == 0 || required >= buffer.size()) {
        return std::nullopt;
    }

    return fs::path(std::wstring(buffer.data(), required));
}

void log_cuda_runtime_environment(const fs::path &exe_dir)
{
    info_and_log("CUDA runtime dependency probe:");

    struct DependencyEntry {
        const char *name;
        bool required;
    };

    const std::array<DependencyEntry, 10> dependencies{{
        {"onnxruntime_providers_cuda.dll", true},
        {"cudart64_110.dll", true},
        {"cublas64_11.dll", true},
        {"cublasLt64_11.dll", true},
        {"cudnn64_8.dll", true},
        {"cudnn_ops_infer64_8.dll", true},
        {"cudnn_cnn_infer64_8.dll", true},
        {"cufft64_10.dll", true},
        {"nvrtc64_112_0.dll", true},
        {"nvrtc-builtins64_114.dll", true}
    }};

    for (const auto &dependency : dependencies) {
        const fs::path local_path = exe_dir / dependency.name;
        std::error_code ec;
        if (fs::exists(local_path, ec) && !ec) {
            info_and_log("  FOUND next_to_exe: " + std::string(dependency.name));
            continue;
        }

        const auto resolved = search_dll_in_loader_path(fs::path(dependency.name).wstring());
        if (resolved) {
            info_and_log(
                "  FOUND on_loader_path: " + std::string(dependency.name) +
                " -> " + resolved->string());
            continue;
        }

        if (dependency.required) {
            warn_and_log(
                "Missing required CUDA runtime dependency for gpu_cuda: " +
                std::string(dependency.name));
        } else {
            info_and_log(
                "  Optional CUDA dependency not found on loader path: " +
                std::string(dependency.name));
        }
    }

    const fs::path provider_path = exe_dir / "onnxruntime_providers_cuda.dll";
    std::error_code provider_ec;
    if (!(fs::exists(provider_path, provider_ec) && !provider_ec)) {
        warn_and_log("onnxruntime_providers_cuda.dll is not present next to the executable.");
        return;
    }

    SetLastError(ERROR_SUCCESS);
    HMODULE provider_module = LoadLibraryExW(
        provider_path.wstring().c_str(),
        nullptr,
        LOAD_WITH_ALTERED_SEARCH_PATH);
    if (provider_module == nullptr) {
        const DWORD load_error = GetLastError();
        warn_and_log(
            "CUDA provider probe failed for onnxruntime_providers_cuda.dll. "
            "LoadLibraryExW error " + std::to_string(load_error) + ": " +
            format_win32_error_message(load_error));
        return;
    }

    FreeLibrary(provider_module);
    info_and_log("CUDA provider probe succeeded: onnxruntime_providers_cuda.dll");
}

void log_dxgi_adapters_for_body_tracking()
{
    IDXGIFactory1 *factory = nullptr;
    const HRESULT create_result = CreateDXGIFactory1(__uuidof(IDXGIFactory1), reinterpret_cast<void **>(&factory));
    if (FAILED(create_result) || factory == nullptr) {
        warn_and_log("Failed to enumerate DXGI adapters for body tracking.");
        return;
    }

    const auto release_factory = std::unique_ptr<std::remove_pointer_t<IDXGIFactory1>, void(*)(IDXGIFactory1 *)>(
        factory,
        [](IDXGIFactory1 *ptr) {
            if (ptr != nullptr) {
                ptr->Release();
            }
        });

    info_and_log("DXGI adapters for DirectML/CUDA selection:");
    for (UINT adapter_index = 0;; ++adapter_index) {
        IDXGIAdapter *adapter = nullptr;
        const HRESULT enum_result = factory->EnumAdapters(adapter_index, &adapter);
        if (enum_result == DXGI_ERROR_NOT_FOUND) {
            break;
        }
        if (FAILED(enum_result) || adapter == nullptr) {
            warn_and_log("Failed to enumerate a DXGI adapter entry at index " + std::to_string(adapter_index) + ".");
            continue;
        }

        const auto release_adapter = std::unique_ptr<std::remove_pointer_t<IDXGIAdapter>, void(*)(IDXGIAdapter *)>(
            adapter,
            [](IDXGIAdapter *ptr) {
                if (ptr != nullptr) {
                    ptr->Release();
                }
            });

        DXGI_ADAPTER_DESC desc{};
        if (FAILED(adapter->GetDesc(&desc))) {
            warn_and_log("Failed to read DXGI adapter description at index " + std::to_string(adapter_index) + ".");
            continue;
        }

        std::ostringstream oss;
        oss << "  adapter[" << adapter_index << "] name=" << narrow_from_wide(desc.Description)
            << " vendor_id=" << desc.VendorId
            << " dedicated_video_memory_mb=" << (desc.DedicatedVideoMemory / (1024ull * 1024ull));
        if (adapter_index == 0) {
            oss << " <- gpu_device_id=0";
        }
        info_and_log(oss.str());
    }
}

std::vector<KinectDeviceIdentity> enumerate_kinect_devices(
    uint32_t installed_device_count,
    bool suppress_open_failures = false)
{
    std::vector<KinectDeviceIdentity> devices;
    devices.reserve(installed_device_count);
    for (uint32_t index = 0; index < installed_device_count; ++index) {
        k4a_device_t device = nullptr;
        if (k4a_device_open(index, &device) != K4A_RESULT_SUCCEEDED) {
            if (!suppress_open_failures) {
                warn_and_log("Failed to open Azure Kinect during enumeration (index " + std::to_string(index) + ").");
            }
            continue;
        }

        try {
            devices.push_back(KinectDeviceIdentity{index, get_kinect_serial_number(device)});
        } catch (...) {
            k4a_device_close(device);
            throw;
        }

        k4a_device_close(device);
    }
    return devices;
}

std::optional<std::string> find_device_serial_by_index(
    const std::vector<KinectDeviceIdentity> &devices,
    uint32_t index)
{
    for (const auto &device : devices) {
        if (device.index == index) {
            return device.serial;
        }
    }
    return std::nullopt;
}

std::optional<uint32_t> find_device_index_by_serial(
    const std::vector<KinectDeviceIdentity> &devices,
    const std::string &serial)
{
    for (const auto &device : devices) {
        if (device.serial == serial) {
            return device.index;
        }
    }
    return std::nullopt;
}

std::string format_device_list(const std::vector<KinectDeviceIdentity> &devices)
{
    if (devices.empty()) {
        return "(none)";
    }

    std::ostringstream oss;
    for (size_t i = 0; i < devices.size(); ++i) {
        if (i > 0) {
            oss << ", ";
        }
        oss << "index " << devices[i].index << " = " << devices[i].serial;
    }
    return oss.str();
}

uint32_t resolve_device_index(
    const std::vector<KinectDeviceIdentity> &devices,
    const std::string &requested_serial,
    uint32_t fallback_index,
    const std::string &role_name)
{
    if (!requested_serial.empty()) {
        if (auto matched_index = find_device_index_by_serial(devices, requested_serial)) {
            return *matched_index;
        }
        throw std::runtime_error(
            "Requested " + role_name + " Kinect serial was not found: " + requested_serial +
            " | available: " + format_device_list(devices));
    }

    for (const auto &device : devices) {
        if (device.index == fallback_index) {
            return fallback_index;
        }
    }

    throw std::runtime_error(
        "Requested fallback " + role_name + " Kinect index was not found: " + std::to_string(fallback_index) +
        " | available: " + format_device_list(devices));
}

k4abt_tracker_processing_mode_t to_k4abt_processing_mode(const std::string &body_tracking_mode)
{
    if (body_tracking_mode == "gpu") {
        return K4ABT_TRACKER_PROCESSING_MODE_GPU;
    }
    if (body_tracking_mode == "cpu") {
        return K4ABT_TRACKER_PROCESSING_MODE_CPU;
    }
    if (body_tracking_mode == "gpu_cuda") {
        return K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
    }
    if (body_tracking_mode == "gpu_tensorrt") {
        return K4ABT_TRACKER_PROCESSING_MODE_GPU_TENSORRT;
    }
    if (body_tracking_mode == "gpu_directml") {
        return K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
    }
    throw std::runtime_error("Unsupported body_tracking_mode: " + body_tracking_mode);
}

std::vector<std::string> build_body_tracking_mode_candidates(const std::string &requested_mode)
{
    std::vector<std::string> candidates;
    const auto push_unique = [&](const std::string &mode) {
        if (std::find(candidates.begin(), candidates.end(), mode) == candidates.end()) {
            candidates.push_back(mode);
        }
    };

    push_unique(requested_mode);

    if (requested_mode == "gpu_cuda" || requested_mode == "gpu_tensorrt") {
        push_unique("gpu_directml");
        push_unique("gpu");
        push_unique("cpu");
        return candidates;
    }

    if (requested_mode == "gpu_directml") {
        push_unique("gpu");
        push_unique("cpu");
        return candidates;
    }

    if (requested_mode == "gpu") {
        push_unique("gpu_directml");
        push_unique("cpu");
        return candidates;
    }

    push_unique("cpu");
    return candidates;
}

const char *wired_sync_mode_to_string(k4a_wired_sync_mode_t mode)
{
    switch (mode) {
    case K4A_WIRED_SYNC_MODE_STANDALONE:
        return "standalone";
    case K4A_WIRED_SYNC_MODE_MASTER:
        return "master";
    case K4A_WIRED_SYNC_MODE_SUBORDINATE:
        return "subordinate";
    default:
        return "unknown";
    }
}

k4a_device_configuration_t make_aux_device_config(
    const k4a_device_configuration_t &base_config,
    bool standalone_mode,
    uint32_t subordinate_delay_off_master_usec,
    bool synchronized_images_only)
{
    k4a_device_configuration_t aux_config = base_config;
    aux_config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    aux_config.color_resolution = base_config.color_resolution;
    aux_config.synchronized_images_only = synchronized_images_only;
    aux_config.wired_sync_mode = standalone_mode
        ? K4A_WIRED_SYNC_MODE_STANDALONE
        : K4A_WIRED_SYNC_MODE_SUBORDINATE;
    aux_config.subordinate_delay_off_master_usec = standalone_mode
        ? 0
        : subordinate_delay_off_master_usec;
    return aux_config;
}

bool start_aux_camera_with_mode(
    k4a_device_t aux_device,
    const k4a_device_configuration_t &aux_config,
    k4a_calibration_t *aux_calibration_out,
    std::string *failure_stage_out = nullptr)
{
    if (aux_device == nullptr || aux_calibration_out == nullptr) {
        if (failure_stage_out != nullptr) {
            *failure_stage_out = "invalid_args";
        }
        return false;
    }

    if (k4a_device_start_cameras(aux_device, &aux_config) != K4A_RESULT_SUCCEEDED) {
        if (failure_stage_out != nullptr) {
            *failure_stage_out = "start_cameras";
        }
        return false;
    }

    if (k4a_device_get_calibration(
            aux_device,
            aux_config.depth_mode,
            aux_config.color_resolution,
            aux_calibration_out) != K4A_RESULT_SUCCEEDED) {
        if (failure_stage_out != nullptr) {
            *failure_stage_out = "get_calibration";
        }
        k4a_device_stop_cameras(aux_device);
        return false;
    }

    return true;
}

bool restart_aux_camera_with_mode(
    uint32_t fallback_aux_device_index,
    const std::string &preferred_aux_serial,
    uint32_t *resolved_aux_device_index_out,
    k4a_device_t *aux_device_io,
    const k4a_device_configuration_t &aux_config,
    k4a_calibration_t *aux_calibration_out)
{
    if (aux_device_io == nullptr || aux_calibration_out == nullptr || resolved_aux_device_index_out == nullptr) {
        return false;
    }

    if (*aux_device_io != nullptr) {
        k4a_device_stop_cameras(*aux_device_io);
        k4a_device_close(*aux_device_io);
        *aux_device_io = nullptr;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for (int attempt = 1; attempt <= 3; ++attempt) {
        uint32_t open_device_index = fallback_aux_device_index;
        if (!preferred_aux_serial.empty()) {
            const uint32_t installed_device_count = k4a_device_get_installed_count();
            const auto available_devices = enumerate_kinect_devices(installed_device_count, true);
            if (!available_devices.empty()) {
                info_and_log("Available Azure Kinect devices during aux restart: " + format_device_list(available_devices));
            } else {
                warn_and_log("No Azure Kinect devices were discoverable during aux restart enumeration.");
            }
            if (auto matched_index = find_device_index_by_serial(available_devices, preferred_aux_serial)) {
                open_device_index = *matched_index;
            } else {
                warn_and_log(
                    "Preferred auxiliary Kinect serial was not found during restart: " +
                    preferred_aux_serial + ". Falling back to device index " +
                    std::to_string(fallback_aux_device_index) + ".");
            }
        }

        info_and_log(
            "Attempting to restart auxiliary Kinect with mode=" +
            std::string(wired_sync_mode_to_string(aux_config.wired_sync_mode)) +
            ", synchronized_images_only=" + (aux_config.synchronized_images_only ? "true" : "false") +
            ", target_index=" + std::to_string(open_device_index) +
            " (attempt " + std::to_string(attempt) + "/3)");

        if (k4a_device_open(open_device_index, aux_device_io) != K4A_RESULT_SUCCEEDED) {
            warn_and_log(
                "Failed to reopen auxiliary Kinect device at index " +
                std::to_string(open_device_index) + ".");
            *aux_device_io = nullptr;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        std::string failure_stage;
        if (start_aux_camera_with_mode(*aux_device_io, aux_config, aux_calibration_out, &failure_stage)) {
            *resolved_aux_device_index_out = open_device_index;
            return true;
        }

        warn_and_log("Auxiliary Kinect restart attempt failed at stage: " + failure_stage);
        k4a_device_close(*aux_device_io);
        *aux_device_io = nullptr;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return false;
}

std::optional<int64_t> get_capture_depth_timestamp_usec(k4a_capture_t capture)
{
    if (capture == nullptr) {
        return std::nullopt;
    }

    k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == nullptr) {
        return std::nullopt;
    }

    const auto release_depth_image = std::unique_ptr<std::remove_pointer_t<k4a_image_t>, void(*)(k4a_image_t)>(
        depth_image,
        k4a_image_release);
    return static_cast<int64_t>(k4a_image_get_device_timestamp_usec(depth_image));
}

std::string format_optional_depth_timestamp_usec(const std::optional<int64_t> &timestamp_usec)
{
    return timestamp_usec.has_value() ? std::to_string(*timestamp_usec) : "N/A";
}

std::string format_optional_success_age_ms(const std::optional<Clock::time_point> &last_success_time)
{
    if (!last_success_time.has_value()) {
        return "N/A";
    }
    return std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - *last_success_time).count());
}

class LatestCapturePump {
public:
    LatestCapturePump() = default;

    ~LatestCapturePump()
    {
        stop();
    }

    LatestCapturePump(const LatestCapturePump &) = delete;
    LatestCapturePump &operator=(const LatestCapturePump &) = delete;

    bool start(k4a_device_t device, int32_t capture_timeout_ms, std::string name)
    {
        stop();
        if (device == nullptr) {
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            device_ = device;
            capture_timeout_ms_ = capture_timeout_ms;
            name_ = std::move(name);
            stop_requested_ = false;
            latest_capture_ = nullptr;
            last_depth_timestamp_usec_.reset();
            generation_ = 0;
            has_success_ = false;
            last_success_time_ = Clock::time_point{};
            consecutive_timeouts_ = 0;
            consecutive_failures_ = 0;
        }

        worker_ = std::thread([this]() { run(); });
        return true;
    }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_requested_ = true;
        }

        if (worker_.joinable()) {
            worker_.join();
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_capture_ != nullptr) {
            k4a_capture_release(latest_capture_);
            latest_capture_ = nullptr;
        }
        device_ = nullptr;
        generation_ = 0;
        last_depth_timestamp_usec_.reset();
        has_success_ = false;
        last_success_time_ = Clock::time_point{};
        consecutive_timeouts_ = 0;
        consecutive_failures_ = 0;
    }

    bool get_latest_capture(
        k4a_capture_t *capture_out,
        uint64_t *generation_out = nullptr,
        std::optional<int64_t> *depth_timestamp_out = nullptr,
        std::optional<Clock::time_point> *last_success_time_out = nullptr) const
    {
        if (capture_out == nullptr) {
            return false;
        }
        *capture_out = nullptr;

        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_capture_ == nullptr) {
            return false;
        }

        k4a_capture_reference(latest_capture_);
        *capture_out = latest_capture_;
        if (generation_out != nullptr) {
            *generation_out = generation_;
        }
        if (depth_timestamp_out != nullptr) {
            *depth_timestamp_out = last_depth_timestamp_usec_;
        }
        if (last_success_time_out != nullptr) {
            *last_success_time_out = has_success_ ? std::optional<Clock::time_point>(last_success_time_) : std::nullopt;
        }
        return true;
    }

    void get_status(
        uint64_t *generation_out = nullptr,
        std::optional<int64_t> *depth_timestamp_out = nullptr,
        std::optional<Clock::time_point> *last_success_time_out = nullptr,
        int *consecutive_timeouts_out = nullptr,
        int *consecutive_failures_out = nullptr) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation_out != nullptr) {
            *generation_out = generation_;
        }
        if (depth_timestamp_out != nullptr) {
            *depth_timestamp_out = last_depth_timestamp_usec_;
        }
        if (last_success_time_out != nullptr) {
            *last_success_time_out = has_success_ ? std::optional<Clock::time_point>(last_success_time_) : std::nullopt;
        }
        if (consecutive_timeouts_out != nullptr) {
            *consecutive_timeouts_out = consecutive_timeouts_;
        }
        if (consecutive_failures_out != nullptr) {
            *consecutive_failures_out = consecutive_failures_;
        }
    }

private:
    void run()
    {
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

        while (true) {
            k4a_device_t device = nullptr;
            int32_t capture_timeout_ms = 0;
            std::string pump_name;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stop_requested_) {
                    break;
                }
                device = device_;
                capture_timeout_ms = capture_timeout_ms_;
                pump_name = name_;
            }

            if (device == nullptr) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            k4a_capture_t capture = nullptr;
            const k4a_wait_result_t wait_result = k4a_device_get_capture(device, &capture, capture_timeout_ms);
            if (wait_result == K4A_WAIT_RESULT_TIMEOUT) {
                bool should_log_timeout = false;
                int timeout_count = 0;
                uint64_t generation_snapshot = 0;
                std::optional<int64_t> depth_timestamp_snapshot;
                std::optional<Clock::time_point> last_success_time_snapshot;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++consecutive_timeouts_;
                    consecutive_failures_ = 0;
                    timeout_count = consecutive_timeouts_;
                    generation_snapshot = generation_;
                    depth_timestamp_snapshot = last_depth_timestamp_usec_;
                    last_success_time_snapshot = has_success_ ? std::optional<Clock::time_point>(last_success_time_) : std::nullopt;
                    should_log_timeout =
                        has_success_ &&
                        (timeout_count == 5 || timeout_count % 30 == 0);
                }
                if (should_log_timeout) {
                    warn_and_log(
                        pump_name + " capture pump timed out waiting for a new frame. consecutive_timeouts=" +
                        std::to_string(timeout_count) +
                        " last_generation=" + std::to_string(generation_snapshot) +
                        " last_depth_ts_us=" + format_optional_depth_timestamp_usec(depth_timestamp_snapshot) +
                        " last_success_age_ms=" + format_optional_success_age_ms(last_success_time_snapshot));
                }
                continue;
            }
            if (wait_result != K4A_WAIT_RESULT_SUCCEEDED || capture == nullptr) {
                bool should_log_failure = false;
                int failure_count = 0;
                uint64_t generation_snapshot = 0;
                std::optional<int64_t> depth_timestamp_snapshot;
                std::optional<Clock::time_point> last_success_time_snapshot;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    consecutive_timeouts_ = 0;
                    ++consecutive_failures_;
                    failure_count = consecutive_failures_;
                    generation_snapshot = generation_;
                    depth_timestamp_snapshot = last_depth_timestamp_usec_;
                    last_success_time_snapshot = has_success_ ? std::optional<Clock::time_point>(last_success_time_) : std::nullopt;
                    should_log_failure = (failure_count == 1 || failure_count % 10 == 0);
                }
                if (should_log_failure) {
                    warn_and_log(
                        pump_name + " capture pump received K4A_WAIT_RESULT_FAILED. consecutive_failures=" +
                        std::to_string(failure_count) +
                        " last_generation=" + std::to_string(generation_snapshot) +
                        " last_depth_ts_us=" + format_optional_depth_timestamp_usec(depth_timestamp_snapshot) +
                        " last_success_age_ms=" + format_optional_success_age_ms(last_success_time_snapshot));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            const auto depth_timestamp_usec = get_capture_depth_timestamp_usec(capture);
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_requested_) {
                k4a_capture_release(capture);
                break;
            }
            if (latest_capture_ != nullptr) {
                k4a_capture_release(latest_capture_);
            }
            latest_capture_ = capture;
            last_depth_timestamp_usec_ = depth_timestamp_usec;
            ++generation_;
            has_success_ = true;
            last_success_time_ = Clock::now();
            consecutive_timeouts_ = 0;
            consecutive_failures_ = 0;
        }
    }

    mutable std::mutex mutex_;
    std::thread worker_;
    k4a_device_t device_ = nullptr;
    int32_t capture_timeout_ms_ = 100;
    std::string name_;
    bool stop_requested_ = false;
    k4a_capture_t latest_capture_ = nullptr;
    std::optional<int64_t> last_depth_timestamp_usec_;
    uint64_t generation_ = 0;
    bool has_success_ = false;
    Clock::time_point last_success_time_{};
    int consecutive_timeouts_ = 0;
    int consecutive_failures_ = 0;
};

bool create_body_tracker_with_fallback(
    const k4a_calibration_t &calibration,
    const std::string &tracker_label,
    const std::string &requested_mode,
    int32_t requested_gpu_device_id,
    const std::string &model_path,
    k4abt_tracker_t *tracker_out,
    std::string *selected_mode_out)
{
    if (tracker_out == nullptr || selected_mode_out == nullptr) {
        return false;
    }

    *tracker_out = nullptr;
    selected_mode_out->clear();

    const auto candidates = build_body_tracking_mode_candidates(requested_mode);
    for (size_t i = 0; i < candidates.size(); ++i) {
        const std::string &mode = candidates[i];
        k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
        tracker_config.processing_mode = to_k4abt_processing_mode(mode);
        tracker_config.gpu_device_id = requested_gpu_device_id;
        tracker_config.model_path = model_path.c_str();

        if (i == 0) {
            info_and_log(
                "Attempting " + tracker_label + " body tracker with mode: " + mode +
                ", gpu_device_id=" + std::to_string(requested_gpu_device_id));
        } else {
            warn_and_log(
                "Retrying " + tracker_label + " body tracker with fallback mode: " + mode +
                ", gpu_device_id=" + std::to_string(requested_gpu_device_id));
        }

        if (k4abt_tracker_create(&calibration, tracker_config, tracker_out) == K4A_RESULT_SUCCEEDED) {
            *selected_mode_out = mode;
            return true;
        }

        warn_and_log(tracker_label + " body tracker creation failed with mode: " + mode);
    }

    return false;
}

void shutdown_and_destroy_tracker(
    k4abt_tracker_t *tracker_io,
    const std::string &tracker_label)
{
    if (tracker_io == nullptr || *tracker_io == nullptr) {
        return;
    }

    info_and_log("Shutting down " + tracker_label + " body tracker.");
    k4abt_tracker_shutdown(*tracker_io);
    k4abt_tracker_destroy(*tracker_io);
    *tracker_io = nullptr;
}

bool is_confident_joint(k4abt_joint_confidence_level_t confidence_level)
{
    return confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
}

const char *joint_confidence_to_string(k4abt_joint_confidence_level_t confidence_level)
{
    switch (confidence_level) {
    case K4ABT_JOINT_CONFIDENCE_NONE:
        return "NONE";
    case K4ABT_JOINT_CONFIDENCE_LOW:
        return "LOW";
    case K4ABT_JOINT_CONFIDENCE_MEDIUM:
        return "MEDIUM";
    case K4ABT_JOINT_CONFIDENCE_HIGH:
        return "HIGH";
    default:
        return "UNKNOWN";
    }
}

class FusionTraceWriter {
public:
    void open_if_enabled(const AppConfig &config, const fs::path &config_path)
    {
        if (!config.enable_fusion_trace_csv) {
            return;
        }

        path_ = resolve_output_path(config.fusion_trace_csv_path, config_path);
        std::error_code ec;
        if (!path_.parent_path().empty()) {
            fs::create_directories(path_.parent_path(), ec);
            if (ec) {
                throw std::runtime_error("Failed to create fusion trace directory: " + path_.parent_path().string());
            }
        }

        ofs_.open(path_, std::ios::trunc);
        if (!ofs_) {
            throw std::runtime_error("Failed to open fusion trace CSV: " + path_.string());
        }

        ofs_ << "frame_index,base_capture_generation,aux_capture_generation,"
                "base_depth_ts_us,aux_depth_ts_us,sync_phase_us,sync_phase_error_us,"
                "source,base_conf,aux_body_conf,base_found,aux_body_found,aux_body_used,aux_depth_found,"
                "base_x_mm,base_y_mm,base_z_mm,"
                "aux_body_in_base_x_mm,aux_body_in_base_y_mm,aux_body_in_base_z_mm,"
                "aux_depth_in_base_x_mm,aux_depth_in_base_y_mm,aux_depth_in_base_z_mm,"
                "fused_x_mm,fused_y_mm,fused_z_mm,fused_minus_base_z_mm,"
                "aux_body_match_err_mm,aux_match_err_mm,aux_reason\n";
        enabled_ = true;
    }

    bool enabled() const
    {
        return enabled_;
    }

    const fs::path &path() const
    {
        return path_;
    }

    void write_row(
        uint64_t frame_index,
        uint64_t base_capture_generation,
        uint64_t aux_capture_generation,
        const std::optional<int64_t> &base_depth_ts_us,
        const std::optional<int64_t> &aux_depth_ts_us,
        const std::optional<int64_t> &sync_phase_us,
        const std::optional<int64_t> &sync_phase_error_us,
        const std::string &source,
        const std::optional<JointSample3D> &base_ear,
        const std::optional<JointSample3D> &aux_ear,
        const std::optional<std::array<double, 3>> &aux_body_in_base,
        bool aux_body_used,
        const std::optional<double> &aux_body_match_error_mm,
        const std::optional<std::array<double, 3>> &aux_depth_in_base,
        const std::optional<AuxDepthSample> &aux_depth_sample,
        const std::optional<std::array<double, 3>> &fused_ear,
        const AuxDepthDebugInfo &aux_depth_debug)
    {
        if (!enabled_ || !ofs_) {
            return;
        }

        ofs_ << frame_index << ','
             << base_capture_generation << ','
             << aux_capture_generation << ',';
        append_optional_int64(base_depth_ts_us);
        ofs_ << ',';
        append_optional_int64(aux_depth_ts_us);
        ofs_ << ',';
        append_optional_int64(sync_phase_us);
        ofs_ << ',';
        append_optional_int64(sync_phase_error_us);
        ofs_ << ','
             << csv_escape(source) << ','
             << csv_escape(base_ear.has_value() ? joint_confidence_to_string(base_ear->confidence_level) : "MISSING") << ','
             << csv_escape(aux_ear.has_value() ? joint_confidence_to_string(aux_ear->confidence_level) : "MISSING") << ','
             << (base_ear.has_value() ? "1" : "0") << ','
             << (aux_body_in_base.has_value() ? "1" : "0") << ','
             << (aux_body_used ? "1" : "0") << ','
             << (aux_depth_sample.has_value() ? "1" : "0") << ',';

        append_optional_vec3(base_ear.has_value() ? std::optional<std::array<double, 3>>(base_ear->position) : std::nullopt);
        ofs_ << ',';
        append_optional_vec3(aux_body_in_base);
        ofs_ << ',';
        append_optional_vec3(aux_depth_in_base);
        ofs_ << ',';
        append_optional_vec3(fused_ear);
        ofs_ << ',';
        if (fused_ear.has_value() && base_ear.has_value()) {
            append_value((*fused_ear)[2] - base_ear->position[2]);
        }
        ofs_ << ',';
        if (aux_body_match_error_mm.has_value()) {
            append_value(*aux_body_match_error_mm);
        }
        ofs_ << ',';
        if (aux_depth_sample.has_value()) {
            append_value(aux_depth_sample->spatial_error_mm);
        }
        ofs_ << ','
             << csv_escape(aux_depth_debug.failure_reason)
             << '\n';
    }

private:
    static std::string csv_escape(const std::string &value)
    {
        std::string escaped = "\"";
        for (char ch : value) {
            if (ch == '"') {
                escaped += "\"\"";
            } else {
                escaped += ch;
            }
        }
        escaped += '"';
        return escaped;
    }

    void append_optional_int64(const std::optional<int64_t> &value)
    {
        if (value.has_value()) {
            ofs_ << *value;
        }
    }

    void append_value(double value)
    {
        ofs_ << std::fixed << std::setprecision(6) << value;
    }

    void append_vec3(const std::array<double, 3> &value)
    {
        append_value(value[0]);
        ofs_ << ',';
        append_value(value[1]);
        ofs_ << ',';
        append_value(value[2]);
    }

    void append_optional_vec3(const std::optional<std::array<double, 3>> &value)
    {
        if (value.has_value()) {
            append_vec3(*value);
        } else {
            ofs_ << ",,";
        }
    }

    bool enabled_ = false;
    fs::path path_;
    std::ofstream ofs_;
};

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

std::optional<std::array<float, 2>> convert_3d_to_depth_2d(
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
        K4A_CALIBRATION_TYPE_DEPTH,
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
    int radius,
    uint8_t blue = 0,
    uint8_t green = 255,
    uint8_t red = 0)
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
                image[idx + 0] = blue;
                image[idx + 1] = green;
                image[idx + 2] = red;
                image[idx + 3] = 255;
            }
        }
    }
}

std::array<double, 3> transform_aux_to_base(
    const std::array<double, 3> &p_aux,
    const AuxKinectTransform &tf)
{
    std::array<double, 3> p_base{};
    for (int row = 0; row < 3; ++row) {
        p_base[row] = tf.translation_mm[row];
        for (int col = 0; col < 3; ++col) {
            p_base[row] += tf.rotation_matrix[row][col] * p_aux[col];
        }
    }

    return p_base;
}

std::array<double, 3> transform_base_to_aux(
    const std::array<double, 3> &p_base,
    const AuxKinectTransform &tf)
{
    std::array<double, 3> shifted{};
    for (int row = 0; row < 3; ++row) {
        shifted[row] = p_base[row] - tf.translation_mm[row];
    }

    std::array<double, 3> p_aux{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            p_aux[row] += tf.rotation_matrix[col][row] * shifted[col];
        }
    }

    return p_aux;
}

double distance_mm(const std::array<double, 3> &a, const std::array<double, 3> &b)
{
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::array<double, 3> fuse_depth_only_points(
    const std::array<double, 3> &p1,
    const std::array<double, 3> &p2)
{
    return {
        p1[0],
        p1[1],
        p2[2]
    };
}

std::vector<JointSample3D> get_tracked_ear_candidates_from_body_frame(
    k4abt_frame_t body_frame,
    const std::string &tracked_ear)
{
    std::vector<JointSample3D> candidates;
    if (body_frame == nullptr) {
        return candidates;
    }

    const size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
    if (num_bodies == 0) {
        return candidates;
    }

    const k4abt_joint_id_t joint_id =
        (tracked_ear == "right") ? K4ABT_JOINT_EAR_RIGHT : K4ABT_JOINT_EAR_LEFT;
    candidates.reserve(num_bodies);
    for (size_t body_index = 0; body_index < num_bodies; ++body_index) {
        k4abt_skeleton_t skeleton{};
        if (k4abt_frame_get_body_skeleton(body_frame, static_cast<uint32_t>(body_index), &skeleton) != K4A_RESULT_SUCCEEDED) {
            continue;
        }

        const auto &ear_joint = skeleton.joints[joint_id];
        if (ear_joint.confidence_level == K4ABT_JOINT_CONFIDENCE_NONE) {
            continue;
        }

        const auto &ear = ear_joint.position;
        candidates.push_back(JointSample3D{
            {
                static_cast<double>(ear.v[0]),
                static_cast<double>(ear.v[1]),
                static_cast<double>(ear.v[2])
            },
            ear_joint.confidence_level,
            body_index
        });
    }
    return candidates;
}

std::optional<JointSample3D> select_frontmost_ear_candidate(
    const std::vector<JointSample3D> &candidates)
{
    if (candidates.empty()) {
        return std::nullopt;
    }

    const JointSample3D *best = nullptr;
    for (const auto &candidate : candidates) {
        if (candidate.position[2] <= 0.0) {
            continue;
        }
        if (best == nullptr ||
            candidate.position[2] < best->position[2] ||
            (candidate.position[2] == best->position[2] && candidate.confidence_level > best->confidence_level)) {
            best = &candidate;
        }
    }

    if (best != nullptr) {
        return *best;
    }
    return candidates.front();
}

std::optional<EarCandidateInBase> find_nearest_ear_candidate_in_base(
    const std::vector<EarCandidateInBase> &candidates,
    const std::array<double, 3> &reference_in_base)
{
    if (candidates.empty()) {
        return std::nullopt;
    }

    const EarCandidateInBase *best = nullptr;
    double best_error_mm = std::numeric_limits<double>::infinity();
    for (const auto &candidate : candidates) {
        const double candidate_error_mm = distance_mm(candidate.position_in_base, reference_in_base);
        if (candidate_error_mm < best_error_mm) {
            best = &candidate;
            best_error_mm = candidate_error_mm;
        }
    }

    if (best == nullptr) {
        return std::nullopt;
    }
    return *best;
}

std::optional<AuxDepthSample> sample_aux_depth_point_for_base_joint(
    const k4a_calibration_t &aux_calibration,
    k4a_capture_t aux_capture,
    const std::array<double, 3> &base_joint,
    const AuxKinectTransform &aux_tf,
    AuxDepthDebugInfo *debug_out = nullptr)
{
    if (debug_out != nullptr) {
        *debug_out = AuxDepthDebugInfo{};
    }

    if (aux_capture == nullptr) {
        if (debug_out != nullptr) {
            debug_out->failure_reason = "no_aux_capture";
        }
        return std::nullopt;
    }

    k4a_image_t aux_depth_image = k4a_capture_get_depth_image(aux_capture);
    if (aux_depth_image == nullptr) {
        if (debug_out != nullptr) {
            debug_out->failure_reason = "no_aux_depth_image";
        }
        return std::nullopt;
    }

    const auto release_depth_image = std::unique_ptr<std::remove_pointer_t<k4a_image_t>, void(*)(k4a_image_t)>(
        aux_depth_image,
        k4a_image_release);

    const auto base_in_aux = transform_base_to_aux(base_joint, aux_tf);
    if (debug_out != nullptr) {
        debug_out->predicted_aux_point_3d = base_in_aux;
    }
    if (base_in_aux[2] <= 0.0) {
        if (debug_out != nullptr) {
            debug_out->failure_reason = "base_point_behind_aux";
        }
        return std::nullopt;
    }

    const auto predicted_aux_depth_2d = convert_3d_to_depth_2d(aux_calibration, base_in_aux);
    if (debug_out != nullptr) {
        debug_out->predicted_depth_2d = predicted_aux_depth_2d;
    }
    if (!predicted_aux_depth_2d.has_value()) {
        if (debug_out != nullptr) {
            debug_out->failure_reason = "depth_projection_failed";
        }
        return std::nullopt;
    }

    const int depth_width = k4a_image_get_width_pixels(aux_depth_image);
    const int depth_height = k4a_image_get_height_pixels(aux_depth_image);
    const int stride_pixels = k4a_image_get_stride_bytes(aux_depth_image) / static_cast<int>(sizeof(uint16_t));
    const auto *depth_buffer = reinterpret_cast<const uint16_t *>(k4a_image_get_buffer(aux_depth_image));
    if (debug_out != nullptr) {
        debug_out->depth_width = depth_width;
        debug_out->depth_height = depth_height;
    }
    if (depth_buffer == nullptr || depth_width <= 0 || depth_height <= 0 || stride_pixels <= 0) {
        if (debug_out != nullptr) {
            debug_out->failure_reason = "invalid_depth_buffer";
        }
        return std::nullopt;
    }

    const int center_x = static_cast<int>(std::llround((*predicted_aux_depth_2d)[0]));
    const int center_y = static_cast<int>(std::llround((*predicted_aux_depth_2d)[1]));
    if (debug_out != nullptr) {
        debug_out->center_x = center_x;
        debug_out->center_y = center_y;
    }
    constexpr int kSearchRadius = 14;
    constexpr double kMaxAcceptedSpatialErrorMm = 450.0;

    double best_spatial_error = std::numeric_limits<double>::infinity();
    int best_x = -1;
    int best_y = -1;
    std::array<double, 3> best_aux_point_3d{};
    int nonzero_depth_candidates = 0;
    int valid_3d_candidates = 0;

    for (int y = center_y - kSearchRadius; y <= center_y + kSearchRadius; ++y) {
        if (y < 0 || y >= depth_height) {
            continue;
        }
        for (int x = center_x - kSearchRadius; x <= center_x + kSearchRadius; ++x) {
            if (x < 0 || x >= depth_width) {
                continue;
            }

            const uint16_t depth_mm = depth_buffer[static_cast<size_t>(y) * static_cast<size_t>(stride_pixels) + static_cast<size_t>(x)];
            if (depth_mm == 0) {
                continue;
            }
            ++nonzero_depth_candidates;

            k4a_float2_t source_2d{};
            source_2d.xy.x = static_cast<float>(x);
            source_2d.xy.y = static_cast<float>(y);
            k4a_float3_t candidate_aux_3d{};
            int valid = 0;
            const k4a_result_t rc = k4a_calibration_2d_to_3d(
                &aux_calibration,
                &source_2d,
                static_cast<float>(depth_mm),
                K4A_CALIBRATION_TYPE_DEPTH,
                K4A_CALIBRATION_TYPE_DEPTH,
                &candidate_aux_3d,
                &valid);
            if (rc != K4A_RESULT_SUCCEEDED || valid == 0) {
                continue;
            }
            ++valid_3d_candidates;

            const double dx_mm = static_cast<double>(candidate_aux_3d.xyz.x) - base_in_aux[0];
            const double dy_mm = static_cast<double>(candidate_aux_3d.xyz.y) - base_in_aux[1];
            const double dz_mm = static_cast<double>(candidate_aux_3d.xyz.z) - base_in_aux[2];
            const double spatial_error = std::sqrt(dx_mm * dx_mm + dy_mm * dy_mm + dz_mm * dz_mm);

            if (spatial_error < best_spatial_error) {
                best_spatial_error = spatial_error;
                best_x = x;
                best_y = y;
                best_aux_point_3d = {
                    static_cast<double>(candidate_aux_3d.xyz.x),
                    static_cast<double>(candidate_aux_3d.xyz.y),
                    static_cast<double>(candidate_aux_3d.xyz.z)
                };
            }
        }
    }

    if (debug_out != nullptr) {
        debug_out->nonzero_depth_candidates = nonzero_depth_candidates;
        debug_out->valid_3d_candidates = valid_3d_candidates;
        debug_out->best_spatial_error_mm = best_spatial_error;
    }

    if (best_x < 0) {
        if (debug_out != nullptr) {
            debug_out->failure_reason =
                (nonzero_depth_candidates == 0) ? "no_nonzero_depth_in_window" : "no_valid_3d_candidate";
        }
        return std::nullopt;
    }
    if (best_spatial_error > kMaxAcceptedSpatialErrorMm) {
        if (debug_out != nullptr) {
            debug_out->failure_reason = "best_spatial_error_above_threshold";
        }
        return std::nullopt;
    }

    if (debug_out != nullptr) {
        debug_out->failure_reason = "found";
    }
    return AuxDepthSample{
        base_in_aux,
        best_aux_point_3d,
        *predicted_aux_depth_2d,
        {
            static_cast<float>(best_x),
            static_cast<float>(best_y)
        },
        best_spatial_error
    };
}

bool get_capture_only(
    k4a_device_t device,
    int32_t capture_timeout_ms,
    k4a_capture_t *capture_out)
{
    *capture_out = nullptr;

    if (device == nullptr) {
        return false;
    }

    if (k4a_device_get_capture(device, capture_out, capture_timeout_ms) != K4A_WAIT_RESULT_SUCCEEDED) {
        return false;
    }

    return true;
}

bool advance_body_tracker_for_latest_capture(
    k4abt_tracker_t tracker,
    k4a_capture_t capture,
    uint64_t capture_generation,
    uint64_t *last_enqueued_generation_io,
    k4abt_frame_t *body_frame_out)
{
    if (body_frame_out == nullptr) {
        return false;
    }
    *body_frame_out = nullptr;

    if (tracker == nullptr) {
        return true;
    }

    constexpr int32_t kTrackerPollTimeoutMs = 0;
    if (capture != nullptr &&
        last_enqueued_generation_io != nullptr &&
        capture_generation != 0 &&
        capture_generation != *last_enqueued_generation_io) {
        if (k4abt_tracker_enqueue_capture(tracker, capture, kTrackerPollTimeoutMs) == K4A_WAIT_RESULT_SUCCEEDED) {
            *last_enqueued_generation_io = capture_generation;
        }
    }

    if (k4abt_tracker_pop_result(tracker, body_frame_out, kTrackerPollTimeoutMs) != K4A_WAIT_RESULT_SUCCEEDED) {
        return true;
    }

    return true;
}

bool align_capture_pair_by_depth_timestamp(
    k4a_device_t base_device,
    k4a_device_t aux_device,
    int32_t capture_timeout_ms,
    int64_t max_timestamp_delta_usec,
    k4a_capture_t *base_capture_io,
    k4a_capture_t *aux_capture_io,
    int64_t *delta_out)
{
    if (base_capture_io == nullptr || aux_capture_io == nullptr) {
        return false;
    }
    if (*base_capture_io == nullptr || *aux_capture_io == nullptr) {
        return false;
    }

    constexpr int kMaxAlignAttempts = 6;
    for (int attempt = 0; attempt < kMaxAlignAttempts; ++attempt) {
        const auto base_depth_ts = get_capture_depth_timestamp_usec(*base_capture_io);
        const auto aux_depth_ts = get_capture_depth_timestamp_usec(*aux_capture_io);
        if (!base_depth_ts.has_value() || !aux_depth_ts.has_value()) {
            return false;
        }

        const int64_t delta_us = *aux_depth_ts - *base_depth_ts;
        if (delta_out != nullptr) {
            *delta_out = delta_us;
        }
        if (std::llabs(delta_us) <= max_timestamp_delta_usec) {
            return true;
        }

        if (delta_us < 0) {
            k4a_capture_release(*aux_capture_io);
            *aux_capture_io = nullptr;
            if (!get_capture_only(aux_device, capture_timeout_ms, aux_capture_io)) {
                return false;
            }
        } else {
            k4a_capture_release(*base_capture_io);
            *base_capture_io = nullptr;
            if (!get_capture_only(base_device, capture_timeout_ms, base_capture_io)) {
                return false;
            }
        }
    }

    return false;
}

std::optional<int64_t> compute_median_delta_us(
    const std::array<int64_t, 5> &samples,
    int sample_count)
{
    if (sample_count <= 0) {
        return std::nullopt;
    }

    auto sorted = samples;
    std::sort(sorted.begin(), sorted.begin() + sample_count);
    return sorted[static_cast<size_t>(sample_count / 2)];
}

int64_t camera_fps_to_frame_period_usec(k4a_fps_t camera_fps)
{
    switch (camera_fps) {
    case K4A_FRAMES_PER_SECOND_30:
        return 33333;
    case K4A_FRAMES_PER_SECOND_15:
        return 66667;
    case K4A_FRAMES_PER_SECOND_5:
        return 200000;
    default:
        return 33333;
    }
}

int64_t normalize_delta_to_frame_phase(int64_t delta_us, int64_t frame_period_usec)
{
    if (frame_period_usec <= 0) {
        return delta_us;
    }

    int64_t phase_us = delta_us % frame_period_usec;
    const int64_t half_period_usec = frame_period_usec / 2;
    if (phase_us > half_period_usec) {
        phase_us -= frame_period_usec;
    } else if (phase_us < -half_period_usec) {
        phase_us += frame_period_usec;
    }
    return phase_us;
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        const std::string config_arg = (argc > 1) ? argv[1] : "track_config_2.json";
        const fs::path config_path = resolve_config_path(config_arg);
        reset_runtime_log();
        append_runtime_log("=== track_test_cpp_2cam start ===");
        std::cout << "Loading configuration...\n";
        std::cout << "Config path: " << config_path.string() << "\n";
        append_runtime_log("Config path: " + config_path.string());
        std::cout << "Current working directory: " << fs::current_path().string() << "\n";
        append_runtime_log("Current working directory: " + fs::current_path().string());
        const DWORD process_id = GetCurrentProcessId();
        std::cout << "Process ID: " << process_id << "\n";
        append_runtime_log("Process ID: " + std::to_string(process_id));
        AppConfig config = load_config(config_path.string());

        AuxKinectTransform aux_tf{
            config.aux_translation_mm,
            config.aux_rotation_matrix
        };

        std::cout << "Initializing PTU and Kinects...\n";
        std::cout << "PTU ports : " << config.pal1.port << " " << config.pal2.port << " " << config.pal3.port << '\n';
        std::cout << "PTU enabled: " << (config.enable_ptu ? "true" : "false") << '\n';
        std::cout << "Body tracking enabled: " << (config.enable_body_tracking ? "true" : "false") << '\n';
        std::cout << "Aux body tracking enabled: " << (config.enable_aux_body_tracking ? "true" : "false") << '\n';
        std::cout << "Max aux body match error [mm]: " << config.max_aux_body_match_error_mm << '\n';
        std::cout << "Body tracking mode: " << config.body_tracking_mode << '\n';
        std::cout << "Body tracking gpu_device_id: " << config.body_tracking_gpu_device_id << '\n';
        std::cout << "Body tracking model asset: " << config.body_tracking_model_path << '\n';
        std::cout << "Tracked ear: " << config.tracked_ear << '\n';
        std::cout << "Tracked person camera: " << config.tracked_person_camera << '\n';
        std::cout << "Fusion mode: " << config.fusion_mode;
        if (config.fusion_mode == "depth_only") {
            std::cout << " (base x/y + aux z)";
        } else {
            std::cout << " (use transformed aux 3D point when available)";
        }
        std::cout << '\n';
        std::cout << "Aux view mode: " << config.aux_view_mode << '\n';
        std::cout << "Aux depth correction: enabled when auxiliary depth is available\n";
        std::cout << "Aux stream mode: color stays enabled internally for wired sync and capture when needed\n";
        std::cout << "Aux synchronized_images_only: "
                  << (config.aux_synchronized_images_only ? "true" : "false");
        if (config.aux_synchronized_images_only) {
            std::cout << " (prefer paired color/depth captures for aux)\n";
        } else {
            std::cout << " (prefer depth continuity over paired color/depth captures)\n";
        }
        std::cout << "Allow aux unsynced fallback: " << (config.allow_aux_unsynced_fallback ? "true" : "false") << '\n';
        std::cout << "Image capture enabled: " << (config.enable_image_capture ? "true" : "false") << '\n';
        if (config.enable_image_capture) {
            std::cout << "Image capture output dir: " << config.capture_output_dir << '\n';
            std::cout << "Capture aux depth image: " << (config.capture_save_aux_depth ? "true" : "false") << '\n';
        }
        std::cout << "Fusion trace CSV enabled: " << (config.enable_fusion_trace_csv ? "true" : "false") << '\n';
        if (config.enable_fusion_trace_csv) {
            std::cout << "Fusion trace CSV path: " << config.fusion_trace_csv_path << '\n';
        }
        std::cout << "capture_timeout_ms: " << config.capture_timeout_ms << '\n';
        std::cout << "body_tracking_timeout_ms(config): " << config.body_tracking_timeout_ms << '\n';
        std::cout << "body_tracking_poll_mode: nonblocking (0 ms in 2-camera mode)\n";
        std::cout << "Base Kinect serial: "
                  << (config.base_kinect_serial.empty() ? "(auto:index 0)" : config.base_kinect_serial) << '\n';
        std::cout << "Aux Kinect serial: "
                  << (config.aux_kinect_serial.empty() ? "(auto:index 1)" : config.aux_kinect_serial) << '\n';
        std::cout << "Aux Kinect translation [mm]: "
                  << aux_tf.translation_mm[0] << ", "
                  << aux_tf.translation_mm[1] << ", "
                  << aux_tf.translation_mm[2] << '\n';
        std::cout << "Aux Kinect rotation matrix:\n";
        for (const auto &row : aux_tf.rotation_matrix) {
            std::cout << "  [" << row[0] << ", " << row[1] << ", " << row[2] << "]\n";
        }
        std::cout << "subordinate_delay_off_master_usec: "
                  << config.subordinate_delay_off_master_usec << '\n';
        const uint32_t installed_device_count = k4a_device_get_installed_count();
        std::cout << "Installed Azure Kinect devices: " << installed_device_count << '\n';
        append_runtime_log("Installed Azure Kinect devices: " + std::to_string(installed_device_count));
        const auto available_devices = enumerate_kinect_devices(installed_device_count);
        const std::string available_devices_text = format_device_list(available_devices);
        std::cout << "Detected Azure Kinect devices: " << available_devices_text << '\n';
        append_runtime_log("Detected Azure Kinect devices: " + available_devices_text);
        log_dxgi_adapters_for_body_tracking();
        const uint32_t base_device_index = resolve_device_index(
            available_devices,
            config.base_kinect_serial,
            0,
            "base");
        const uint32_t aux_device_index = resolve_device_index(
            available_devices,
            config.aux_kinect_serial,
            1,
            "aux");
        const std::string base_device_serial_in_use =
            find_device_serial_by_index(available_devices, base_device_index).value_or(config.base_kinect_serial);
        uint32_t current_aux_device_index = aux_device_index;
        std::string aux_device_serial_in_use =
            find_device_serial_by_index(available_devices, aux_device_index).value_or(config.aux_kinect_serial);
        if (base_device_index == aux_device_index) {
            throw std::runtime_error(
                "Base Kinect and auxiliary Kinect resolved to the same device index: " +
                std::to_string(base_device_index));
        }
        if ((config.base_kinect_serial.empty() || config.aux_kinect_serial.empty()) && installed_device_count >= 2) {
            warn_and_log(
                "Kinect serials are not fully pinned in the config. "
                "If Windows changes device index order, checkerboard extrinsics may be applied to the wrong camera.");
        }
        std::cout << "Base Kinect role: MASTER (device index " << base_device_index
                  << ", serial " << (base_device_serial_in_use.empty() ? "unknown" : base_device_serial_in_use) << ")" << '\n';
        std::cout << "Aux Kinect role: SUBORDINATE (device index " << aux_device_index
                  << ", serial " << (aux_device_serial_in_use.empty() ? "unknown" : aux_device_serial_in_use) << ")" << '\n';
        append_runtime_log("Base Kinect device index: " + std::to_string(base_device_index));
        append_runtime_log("Aux Kinect device index: " + std::to_string(aux_device_index));
        append_runtime_log("Base Kinect serial in use: " + base_device_serial_in_use);
        append_runtime_log("Aux Kinect serial in use: " + aux_device_serial_in_use);

        std::unique_ptr<PTUController> ptu1;
        std::unique_ptr<PTUController> ptu2;
        std::unique_ptr<PTUController> ptu3;
        if (config.enable_ptu) {
            try {
                ptu1 = std::make_unique<PTUController>(config.pal1.port);
                ptu2 = std::make_unique<PTUController>(config.pal2.port);
                ptu3 = std::make_unique<PTUController>(config.pal3.port);
                std::cout << "PTU controllers initialized.\n";
            } catch (const std::exception &e) {
                warn_and_log(e.what());
                warn_and_log("Continuing without PTU control.");
                ptu1.reset();
                ptu2.reset();
                ptu3.reset();
            }
        } else {
            std::cout << "PTU control disabled by config.\n";
        }

        k4a_device_t base_device = nullptr;
        k4a_device_t aux_device = nullptr;
        check_k4a(
            k4a_device_open(base_device_index, &base_device),
            "Failed to open base Azure Kinect device (index " + std::to_string(base_device_index) + ")");

        bool aux_camera_enabled = true;
        if (k4a_device_open(aux_device_index, &aux_device) != K4A_RESULT_SUCCEEDED) {
            aux_camera_enabled = false;
            aux_device = nullptr;
            warn_and_log(
                "Failed to open auxiliary Azure Kinect device (index " + std::to_string(aux_device_index) + ").");
            warn_and_log("Continuing with base Kinect only.");
        }

        k4a_device_configuration_t base_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        base_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        base_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        base_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        base_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        base_config.synchronized_images_only = true;
        base_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;

        bool aux_is_unsynced = false;
        bool aux_unsynced_fallback_attempted = false;
        k4a_calibration_t aux_calibration{};
        k4a_device_configuration_t aux_config = make_aux_device_config(
            base_config,
            false,
            config.subordinate_delay_off_master_usec,
            config.aux_synchronized_images_only);

        // Start the subordinate first so it can wait for the master's sync pulse.
        if (aux_camera_enabled) {
            std::string aux_start_failure_stage;
            if (!start_aux_camera_with_mode(aux_device, aux_config, &aux_calibration, &aux_start_failure_stage)) {
                warn_and_log("Failed to start auxiliary cameras at stage: " + aux_start_failure_stage);
                warn_and_log("Continuing with base Kinect only.");
                k4a_device_close(aux_device);
                aux_device = nullptr;
                aux_camera_enabled = false;
            }
        }
        check_k4a(k4a_device_start_cameras(base_device, &base_config), "Failed to start base cameras");

        k4a_calibration_t base_calibration{};
        check_k4a(
            k4a_device_get_calibration(base_device, base_config.depth_mode, base_config.color_resolution, &base_calibration),
            "Failed to get base calibration");

        const fs::path body_tracking_model_path =
            resolve_asset_path(config.body_tracking_model_path, config_path);
        const std::string body_tracking_model_path_string = body_tracking_model_path.string();
        std::cout << "Body tracking model path: " << body_tracking_model_path_string << '\n';
        append_runtime_log("Body tracking model path: " + body_tracking_model_path_string);
        if (config.enable_body_tracking && config.body_tracking_mode == "gpu_cuda") {
            log_cuda_runtime_environment(get_executable_path().parent_path());
        }

        k4abt_tracker_t base_tracker = nullptr;
        k4abt_tracker_t aux_tracker = nullptr;
        auto create_aux_tracker_if_requested = [&](bool restarted) {
            if (!(config.enable_body_tracking &&
                  config.enable_aux_body_tracking &&
                  aux_camera_enabled &&
                  aux_device != nullptr)) {
                return;
            }

            std::string aux_tracker_mode_in_use;
            if (!create_body_tracker_with_fallback(
                    aux_calibration,
                    "auxiliary",
                    config.body_tracking_mode,
                    config.body_tracking_gpu_device_id,
                    body_tracking_model_path_string,
                    &aux_tracker,
                    &aux_tracker_mode_in_use)) {
                warn_and_log("Failed to create auxiliary body tracker.");
                if (restarted) {
                    warn_and_log("Continuing with depth-only correction for the auxiliary Kinect after restart.");
                } else {
                    warn_and_log("Continuing with depth-only correction for the auxiliary Kinect.");
                }
                aux_tracker = nullptr;
                return;
            }

            if (restarted) {
                info_and_log("Aux body tracker recreated after auxiliary restart with mode: " + aux_tracker_mode_in_use);
            } else {
                info_and_log("Aux body tracker created with mode: " + aux_tracker_mode_in_use);
            }
            if (aux_tracker_mode_in_use != config.body_tracking_mode) {
                warn_and_log(
                    "Configured body_tracking_mode '" + config.body_tracking_mode +
                    "' was unavailable for auxiliary tracking. Using fallback mode '" +
                    aux_tracker_mode_in_use + "'.");
            }
        };
        bool aux_body_tracking_requested = false;
        if (config.enable_body_tracking) {
            aux_body_tracking_requested =
                aux_camera_enabled && config.enable_aux_body_tracking;
            std::string base_tracker_mode_in_use;
            if (!create_body_tracker_with_fallback(
                    base_calibration,
                    "base",
                    config.body_tracking_mode,
                    config.body_tracking_gpu_device_id,
                    body_tracking_model_path_string,
                    &base_tracker,
                    &base_tracker_mode_in_use)) {
                warn_and_log("Failed to create base body tracker.");
                warn_and_log("Continuing with color view only for the base Kinect.");
                base_tracker = nullptr;
            } else {
                info_and_log("Base body tracker created with mode: " + base_tracker_mode_in_use);
                if (base_tracker_mode_in_use != config.body_tracking_mode) {
                    warn_and_log(
                        "Configured body_tracking_mode '" + config.body_tracking_mode +
                        "' was unavailable. Using fallback mode '" + base_tracker_mode_in_use + "'.");
                }
            }
            if (aux_body_tracking_requested) {
                create_aux_tracker_if_requested(false);
            } else if (aux_camera_enabled) {
                std::cout << "Auxiliary Kinect will use depth-only correction (no auxiliary body tracker).\n";
                append_runtime_log("Auxiliary Kinect will use depth-only correction (no auxiliary body tracker).");
            }
        } else {
            std::cout << "Body tracking disabled by config.\n";
        }
        std::string effective_tracked_person_camera = config.tracked_person_camera;
        if (effective_tracked_person_camera == "aux" &&
            (!config.enable_body_tracking || !aux_body_tracking_requested || aux_tracker == nullptr)) {
            warn_and_log(
                "tracked_person_camera=aux requested, but auxiliary body tracking is unavailable. "
                "Falling back to base camera frontmost-person selection.");
            effective_tracked_person_camera = "base";
        }
        ImageWindow base_image_window;
        ImageWindow aux_image_window;
        bool base_window_initialized = false;
        bool aux_window_initialized = false;
        constexpr auto kWindowStatusDuration = std::chrono::milliseconds(1800);
        constexpr auto kWindowRestartStatusDuration = std::chrono::milliseconds(4200);
        const auto base_window_size = color_resolution_to_size(base_config.color_resolution);
        if (!base_image_window.create(
                L"Base Azure Kinect [MASTER]",
                base_window_size[0],
                base_window_size[1],
                L"MASTER / Base Kinect")) {
            throw std::runtime_error("Failed to create base image window");
        }
        base_window_initialized = true;
        std::vector<uint8_t> base_startup_display(
            static_cast<size_t>(base_window_size[0]) * static_cast<size_t>(base_window_size[1]) * 4,
            0);
        base_image_window.show_bgra(base_startup_display.data());
        std::cout << "Base Azure Kinect window created." << '\n';
        append_runtime_log("Base Azure Kinect window created.");
        std::vector<uint8_t> aux_startup_display;
        if (aux_device != nullptr) {
            const auto aux_window_size = (config.aux_view_mode == "color")
                ? color_resolution_to_size(aux_config.color_resolution)
                : depth_mode_to_size(aux_config.depth_mode);
            if (!aux_image_window.create(
                    L"Aux Azure Kinect [SUBORDINATE]",
                    aux_window_size[0],
                    aux_window_size[1],
                    (config.aux_view_mode == "color") ? L"SUBORDINATE / Aux Color" : L"SUBORDINATE / Aux Depth")) {
                throw std::runtime_error("Failed to create auxiliary image window");
            }
            aux_window_initialized = true;
            aux_startup_display.assign(
                static_cast<size_t>(aux_window_size[0]) * static_cast<size_t>(aux_window_size[1]) * 4,
                0);
            aux_image_window.show_bgra(aux_startup_display.data());
            std::cout << "Aux Azure Kinect window created." << '\n';
            append_runtime_log("Aux Azure Kinect window created.");
        }
        const auto show_aux_window_status =
            [&](const std::wstring &message,
                std::chrono::milliseconds duration,
                bool clear_to_black) {
                if (!aux_window_initialized) {
                    return;
                }
                aux_image_window.show_status(message, duration);
                if (clear_to_black && !aux_startup_display.empty()) {
                    aux_image_window.show_bgra(aux_startup_display.data());
                }
            };

        std::optional<ImageCaptureSession> image_capture_session;
        if (config.enable_image_capture) {
            image_capture_session = create_image_capture_session(config, config_path);
            info_and_log("Image capture session directory: " + image_capture_session->session_dir.string());
            info_and_log("Press 'c' to save the current base/aux image pair.");
        }

        FusionTraceWriter fusion_trace_writer;
        fusion_trace_writer.open_if_enabled(config, config_path);
        if (fusion_trace_writer.enabled()) {
            info_and_log("Fusion trace CSV: " + fusion_trace_writer.path().string());
        }

        LatestCapturePump base_capture_pump;
        base_capture_pump.start(base_device, 100, "base");
        info_and_log("Base capture pump started.");

        LatestCapturePump aux_capture_pump;
        if (aux_camera_enabled && aux_device != nullptr) {
            aux_capture_pump.start(aux_device, 100, "aux");
            info_and_log("Aux capture pump started.");
        }

        TrackingController pal1(ptu1.get(), config.pal1.vc, config.update_interval_sec, config.pt_factor);
        TrackingController pal2(ptu2.get(), config.pal2.vc, config.update_interval_sec, config.pt_factor);
        TrackingController pal3(ptu3.get(), config.pal3.vc, config.update_interval_sec, config.pt_factor);

        std::optional<std::array<double, 3>> last_fused_tracked_ear;
        std::optional<std::array<float, 2>> last_fused_tracked_ear_2d;
        std::optional<std::array<float, 2>> last_aux_predicted_ear_2d;
        std::optional<std::array<float, 2>> last_aux_tracked_ear_2d;
        int consecutive_base_capture_failures = 0;
        int consecutive_aux_capture_failures = 0;
        int successful_dual_capture_count = 0;
        bool aux_sync_checklist_logged = false;
        bool aux_subordinate_restart_attempted = false;
        int consecutive_aux_alignment_failures = 0;
        std::array<int64_t, 5> sync_phase_baseline_samples{};
        int sync_phase_baseline_sample_count = 0;
        std::optional<int64_t> sync_phase_baseline_us;
        const int64_t frame_period_usec = camera_fps_to_frame_period_usec(base_config.camera_fps);
        uint64_t last_base_capture_generation_enqueued = 0;
        uint64_t last_aux_capture_generation_enqueued = 0;
        uint64_t last_sync_sample_base_generation = 0;
        uint64_t last_sync_sample_aux_generation = 0;
        uint64_t fusion_trace_frame_index = 0;
        bool pending_aux_subordinate_restart = false;
        bool pending_aux_standalone_restart = false;

        std::cout << "2-Kinect tracking started. Press 'u' to force update, 'q' to quit";
        if (config.enable_image_capture) {
            std::cout << ", 'c' to capture images";
        }
        std::cout << "." << '\n';

        while (true) {
            if ((base_window_initialized && !base_image_window.process_messages()) ||
                (aux_window_initialized && !aux_image_window.process_messages())) {
                break;
            }

            k4a_capture_t base_capture = nullptr;
            k4a_capture_t aux_capture = nullptr;
            k4abt_frame_t base_body_frame = nullptr;
            k4abt_frame_t aux_body_frame = nullptr;

            uint64_t base_capture_generation = 0;
            uint64_t base_pump_generation = 0;
            std::optional<int64_t> base_pump_depth_timestamp;
            std::optional<Clock::time_point> base_last_success_time;
            int base_pump_consecutive_timeouts = 0;
            int base_pump_consecutive_failures = 0;
            base_capture_pump.get_status(
                &base_pump_generation,
                &base_pump_depth_timestamp,
                &base_last_success_time,
                &base_pump_consecutive_timeouts,
                &base_pump_consecutive_failures);
            const bool base_capture_ok = base_capture_pump.get_latest_capture(
                &base_capture,
                &base_capture_generation,
                nullptr,
                nullptr);
            bool base_capture_fresh = base_capture_ok && base_capture != nullptr;
            long long base_capture_age_ms = -1;
            if (base_capture_fresh) {
                if (base_last_success_time.has_value()) {
                    base_capture_age_ms =
                        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - *base_last_success_time).count();
                    if (base_capture_age_ms > 500) {
                        base_capture_fresh = false;
                    }
                } else {
                    base_capture_fresh = false;
                }
            }
            uint64_t aux_capture_generation = 0;
            uint64_t aux_pump_generation = 0;
            std::optional<int64_t> aux_pump_depth_timestamp;
            std::optional<Clock::time_point> aux_last_success_time;
            int aux_pump_consecutive_timeouts = 0;
            int aux_pump_consecutive_failures = 0;
            aux_capture_pump.get_status(
                &aux_pump_generation,
                &aux_pump_depth_timestamp,
                &aux_last_success_time,
                &aux_pump_consecutive_timeouts,
                &aux_pump_consecutive_failures);
            const bool aux_capture_ok = aux_capture_pump.get_latest_capture(
                &aux_capture,
                &aux_capture_generation,
                nullptr,
                nullptr);
            std::optional<int64_t> current_sync_raw_delta_us;
            std::optional<int64_t> current_sync_phase_us;
            std::optional<int64_t> current_sync_phase_error_us;
            bool aux_pair_usable_for_fusion = aux_capture_ok && aux_capture != nullptr;
            const bool sync_sample_generations_advanced =
                base_capture_generation != 0 &&
                aux_capture_generation != 0 &&
                base_capture_generation != last_sync_sample_base_generation &&
                aux_capture_generation != last_sync_sample_aux_generation;
            if (base_capture_fresh && base_capture != nullptr && aux_capture_ok && aux_capture != nullptr &&
                sync_sample_generations_advanced) {
                const auto base_depth_ts = get_capture_depth_timestamp_usec(base_capture);
                const auto aux_depth_ts = get_capture_depth_timestamp_usec(aux_capture);
                if (base_depth_ts.has_value() && aux_depth_ts.has_value()) {
                    last_sync_sample_base_generation = base_capture_generation;
                    last_sync_sample_aux_generation = aux_capture_generation;
                    current_sync_raw_delta_us = *aux_depth_ts - *base_depth_ts;
                    current_sync_phase_us = normalize_delta_to_frame_phase(
                        *current_sync_raw_delta_us,
                        frame_period_usec);

                    if (!sync_phase_baseline_us.has_value()) {
                        if (sync_phase_baseline_sample_count < static_cast<int>(sync_phase_baseline_samples.size())) {
                            sync_phase_baseline_samples[static_cast<size_t>(sync_phase_baseline_sample_count)] =
                                *current_sync_phase_us;
                            ++sync_phase_baseline_sample_count;
                            if (sync_phase_baseline_sample_count <= 5) {
                                info_and_log(
                                    "Sync baseline calibration sample " +
                                    std::to_string(sync_phase_baseline_sample_count) +
                                    "/5: raw_delta_us=" + std::to_string(*current_sync_raw_delta_us) +
                                    " phase_delta_us=" + std::to_string(*current_sync_phase_us));
                            }
                        }
                        if (sync_phase_baseline_sample_count ==
                            static_cast<int>(sync_phase_baseline_samples.size())) {
                            sync_phase_baseline_us = compute_median_delta_us(
                                sync_phase_baseline_samples,
                                sync_phase_baseline_sample_count);
                            if (sync_phase_baseline_us.has_value()) {
                                info_and_log(
                                    "Sync baseline established: phase_delta_us=" +
                                    std::to_string(*sync_phase_baseline_us));
                            }
                        }
                        consecutive_aux_alignment_failures = 0;
                    } else {
                        current_sync_phase_error_us =
                            *current_sync_phase_us - *sync_phase_baseline_us;
                        constexpr int64_t kAllowedSyncPhaseJitterUsec = 5000;
                        if (std::llabs(*current_sync_phase_error_us) > kAllowedSyncPhaseJitterUsec) {
                            aux_pair_usable_for_fusion = false;
                            ++consecutive_aux_alignment_failures;
                            if (consecutive_aux_alignment_failures == 5 ||
                                consecutive_aux_alignment_failures % 30 == 0) {
                                warn_and_log(
                                    "Base/aux sync phase drifted beyond tolerance. raw_delta_us=" +
                                    std::to_string(*current_sync_raw_delta_us) +
                                    " phase_delta_us=" + std::to_string(*current_sync_phase_us) +
                                    " baseline_phase_delta_us=" + std::to_string(*sync_phase_baseline_us) +
                                    " phase_error_us=" + std::to_string(*current_sync_phase_error_us));
                            }
                        } else {
                            consecutive_aux_alignment_failures = 0;
                        }
                    }
                }
            }
            const bool base_body_frame_ok = advance_body_tracker_for_latest_capture(
                base_tracker,
                base_capture,
                base_capture_generation,
                &last_base_capture_generation_enqueued,
                &base_body_frame);
            const bool aux_body_frame_ok = advance_body_tracker_for_latest_capture(
                aux_tracker,
                aux_capture,
                aux_capture_generation,
                &last_aux_capture_generation_enqueued,
                &aux_body_frame);

            if (!base_capture_fresh) {
                ++consecutive_base_capture_failures;
                if (consecutive_base_capture_failures == 5 || consecutive_base_capture_failures % 30 == 0) {
                    std::string message =
                        "Base Kinect capture stopped updating. Check master stream continuity, USB connection, and device access.";
                    if (base_capture_age_ms >= 0) {
                        message += " last_base_age_ms=" + std::to_string(base_capture_age_ms);
                    }
                    message += " capture_generation=" + std::to_string(base_capture_generation);
                    message += " pump_generation=" + std::to_string(base_pump_generation);
                    message += " pump_last_depth_ts_us=" + format_optional_depth_timestamp_usec(base_pump_depth_timestamp);
                    message += " pump_timeouts=" + std::to_string(base_pump_consecutive_timeouts);
                    message += " pump_failures=" + std::to_string(base_pump_consecutive_failures);
                    warn_and_log(message);
                }
            } else {
                consecutive_base_capture_failures = 0;
            }

            const bool aux_capture_received = aux_capture_ok && aux_capture != nullptr;
            bool aux_capture_fresh = aux_capture_received;
            long long aux_capture_age_ms = -1;
            if (aux_capture_received) {
                if (aux_last_success_time.has_value()) {
                    aux_capture_age_ms =
                        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - *aux_last_success_time).count();
                    if (aux_capture_age_ms > 500) {
                        aux_capture_fresh = false;
                    }
                } else {
                    aux_capture_fresh = false;
                }
            }

            if (aux_device != nullptr && !aux_capture_fresh) {
                ++consecutive_aux_capture_failures;
                if (consecutive_aux_capture_failures == 5 || consecutive_aux_capture_failures % 30 == 0) {
                    std::string message =
                        "Aux Kinect capture stopped updating. Check subordinate stream continuity and device access.";
                    if (aux_capture_age_ms >= 0) {
                        message += " last_aux_age_ms=" + std::to_string(aux_capture_age_ms);
                    }
                    message += " capture_generation=" + std::to_string(aux_capture_generation);
                    message += " pump_generation=" + std::to_string(aux_pump_generation);
                    message += " pump_last_depth_ts_us=" + format_optional_depth_timestamp_usec(aux_pump_depth_timestamp);
                    message += " pump_timeouts=" + std::to_string(aux_pump_consecutive_timeouts);
                    message += " pump_failures=" + std::to_string(aux_pump_consecutive_failures);
                    warn_and_log(message);
                    show_aux_window_status(
                        L"Aux capture stalled; viewer is showing the last received depth frame.",
                        kWindowStatusDuration,
                        false);
                }
                if (!aux_sync_checklist_logged &&
                    successful_dual_capture_count == 0 &&
                    consecutive_aux_capture_failures >= 5 &&
                    !aux_is_unsynced) {
                    aux_sync_checklist_logged = true;
                    warn_and_log(
                        "Sync checklist: verify 3.5-mm sync cable is connected MASTER Sync Out -> AUX Sync In, "
                        "start subordinate before master, keep both devices at the same FPS, "
                        "and keep the master color camera enabled.");
                }
                if (!aux_is_unsynced &&
                    successful_dual_capture_count > 0 &&
                    !aux_subordinate_restart_attempted &&
                    consecutive_aux_capture_failures >= 5) {
                    aux_subordinate_restart_attempted = true;
                    pending_aux_subordinate_restart = true;
                    base_image_window.show_status(
                        L"Aux Kinect stalled; restarting subordinate stream.",
                        kWindowRestartStatusDuration);
                    show_aux_window_status(
                        L"Aux capture stalled; restarting subordinate stream...",
                        kWindowRestartStatusDuration,
                        false);
                    warn_and_log(
                        "Aux Kinect capture stopped after synchronization was already established. "
                        "Scheduling auxiliary Kinect restart while keeping subordinate sync mode.");
                }
                if (aux_camera_enabled &&
                    !aux_is_unsynced &&
                    !aux_unsynced_fallback_attempted &&
                    config.allow_aux_unsynced_fallback &&
                    consecutive_aux_capture_failures >= 5) {
                    aux_unsynced_fallback_attempted = true;
                    pending_aux_standalone_restart = true;
                    base_image_window.show_status(
                        L"Aux Kinect stalled; restarting in standalone mode.",
                        kWindowRestartStatusDuration);
                    show_aux_window_status(
                        L"Aux capture stalled; restarting in standalone mode...",
                        kWindowRestartStatusDuration,
                        false);
                    warn_and_log(
                        "Aux Kinect timed out repeatedly in subordinate mode. "
                        "Scheduling auxiliary Kinect restart in standalone mode.");
                }
            } else {
                consecutive_aux_capture_failures = 0;
            }

            if (base_capture_fresh && aux_capture_received) {
                ++successful_dual_capture_count;
                if (successful_dual_capture_count <= 5 || successful_dual_capture_count % 30 == 0) {
                    const auto base_depth_ts = get_capture_depth_timestamp_usec(base_capture);
                    const auto aux_depth_ts = get_capture_depth_timestamp_usec(aux_capture);
                    if (base_depth_ts.has_value() && aux_depth_ts.has_value()) {
                        std::ostringstream oss;
                        oss << "Sync diagnostic: base_depth_ts_us=" << *base_depth_ts
                            << " aux_depth_ts_us=" << *aux_depth_ts
                            << " raw_delta_us=" << *aux_depth_ts - *base_depth_ts;
                        if (current_sync_phase_us.has_value()) {
                            oss << " phase_delta_us=" << *current_sync_phase_us;
                        }
                        if (sync_phase_baseline_us.has_value()) {
                            oss << " baseline_phase_delta_us=" << *sync_phase_baseline_us;
                        } else {
                            oss << " baseline_phase_delta_us=CALIBRATING";
                        }
                        if (current_sync_phase_error_us.has_value()) {
                            oss << " phase_error_us=" << *current_sync_phase_error_us;
                        }
                        oss
                            << " aux_mode=" << wired_sync_mode_to_string(aux_config.wired_sync_mode)
                            << " subordinate_delay_off_master_usec=" << aux_config.subordinate_delay_off_master_usec;
                        info_and_log(oss.str());
                    }
                }
            }

            k4a_image_t base_color_image = nullptr;
            k4a_image_t aux_color_image = nullptr;
            k4a_image_t aux_depth_image = nullptr;
            int color_width = 0;
            int color_height = 0;
            int aux_color_width = 0;
            int aux_color_height = 0;
            int aux_depth_width = 0;
            int aux_depth_height = 0;

            if (base_capture != nullptr) {
                base_color_image = k4a_capture_get_color_image(base_capture);
                if (base_color_image != nullptr) {
                    color_width = k4a_image_get_width_pixels(base_color_image);
                    color_height = k4a_image_get_height_pixels(base_color_image);
                }
            }
            if (aux_capture != nullptr) {
                aux_depth_image = k4a_capture_get_depth_image(aux_capture);
                if (aux_depth_image != nullptr) {
                    aux_depth_width = k4a_image_get_width_pixels(aux_depth_image);
                    aux_depth_height = k4a_image_get_height_pixels(aux_depth_image);
                }
                if (config.aux_view_mode == "color" || config.enable_image_capture) {
                    aux_color_image = k4a_capture_get_color_image(aux_capture);
                    if (aux_color_image != nullptr) {
                        aux_color_width = k4a_image_get_width_pixels(aux_color_image);
                        aux_color_height = k4a_image_get_height_pixels(aux_color_image);
                    }
                }
            }
            const auto base_depth_ts_for_trace = base_capture != nullptr
                ? get_capture_depth_timestamp_usec(base_capture)
                : std::nullopt;
            const auto aux_depth_ts_for_trace = aux_capture != nullptr
                ? get_capture_depth_timestamp_usec(aux_capture)
                : std::nullopt;

            const auto base_ear_candidates =
                (base_capture_fresh && base_body_frame_ok && base_body_frame != nullptr)
                    ? get_tracked_ear_candidates_from_body_frame(base_body_frame, config.tracked_ear)
                    : std::vector<JointSample3D>{};
            const auto aux_ear_candidates =
                (aux_capture_fresh && aux_body_frame_ok && aux_body_frame != nullptr)
                    ? get_tracked_ear_candidates_from_body_frame(aux_body_frame, config.tracked_ear)
                    : std::vector<JointSample3D>{};

            std::vector<EarCandidateInBase> base_ear_candidates_in_base;
            base_ear_candidates_in_base.reserve(base_ear_candidates.size());
            for (const auto &candidate : base_ear_candidates) {
                base_ear_candidates_in_base.push_back(EarCandidateInBase{candidate, candidate.position});
            }

            std::vector<EarCandidateInBase> aux_ear_candidates_in_base;
            aux_ear_candidates_in_base.reserve(aux_ear_candidates.size());
            for (const auto &candidate : aux_ear_candidates) {
                aux_ear_candidates_in_base.push_back(EarCandidateInBase{
                    candidate,
                    transform_aux_to_base(candidate.position, aux_tf)
                });
            }

            std::optional<JointSample3D> base_ear;
            std::optional<JointSample3D> aux_ear;
            std::optional<EarCandidateInBase> selected_aux_ear_in_base;
            if (effective_tracked_person_camera == "aux") {
                aux_ear = select_frontmost_ear_candidate(aux_ear_candidates);
                if (aux_ear.has_value()) {
                    selected_aux_ear_in_base = EarCandidateInBase{
                        *aux_ear,
                        transform_aux_to_base(aux_ear->position, aux_tf)
                    };
                    if (const auto matched_base_ear = find_nearest_ear_candidate_in_base(
                            base_ear_candidates_in_base,
                            selected_aux_ear_in_base->position_in_base)) {
                        base_ear = matched_base_ear->sample;
                    }
                }
            } else {
                base_ear = select_frontmost_ear_candidate(base_ear_candidates);
                if (base_ear.has_value()) {
                    if (const auto matched_aux_ear = find_nearest_ear_candidate_in_base(
                            aux_ear_candidates_in_base,
                            base_ear->position)) {
                        aux_ear = matched_aux_ear->sample;
                        selected_aux_ear_in_base = *matched_aux_ear;
                    }
                }
            }

            const bool base_ear_confident = base_ear.has_value() && is_confident_joint(base_ear->confidence_level);
            const bool aux_ear_confident = aux_ear.has_value() && is_confident_joint(aux_ear->confidence_level);
            auto raw_aux_body_ear_in_base = aux_ear_confident && selected_aux_ear_in_base.has_value()
                ? std::optional<std::array<double, 3>>(selected_aux_ear_in_base->position_in_base)
                : std::nullopt;
            std::optional<std::array<double, 3>> aux_body_ear_in_base;
            std::optional<double> aux_body_match_error_mm;
            bool aux_body_rejected_by_match_gate = false;
            if (base_ear_confident && raw_aux_body_ear_in_base.has_value()) {
                aux_body_match_error_mm = distance_mm(base_ear->position, *raw_aux_body_ear_in_base);
                if (*aux_body_match_error_mm <= config.max_aux_body_match_error_mm) {
                    aux_body_ear_in_base = raw_aux_body_ear_in_base;
                } else {
                    aux_body_rejected_by_match_gate = true;
                }
            }
            auto aux_body_ear_2d = aux_ear_confident
                ? ((config.aux_view_mode == "color")
                    ? convert_3d_to_color_2d(aux_calibration, aux_ear->position)
                    : convert_3d_to_depth_2d(aux_calibration, aux_ear->position))
                : std::nullopt;
            last_aux_predicted_ear_2d = (base_ear_confident && aux_device != nullptr)
                ? ((config.aux_view_mode == "color")
                    ? convert_3d_to_color_2d(aux_calibration, transform_base_to_aux(base_ear->position, aux_tf))
                    : convert_3d_to_depth_2d(aux_calibration, transform_base_to_aux(base_ear->position, aux_tf)))
                : std::nullopt;
            AuxDepthDebugInfo aux_depth_debug{};
            auto aux_depth_sample = (effective_tracked_person_camera == "base" && base_ear_confident && aux_pair_usable_for_fusion)
                ? sample_aux_depth_point_for_base_joint(aux_calibration, aux_capture, base_ear->position, aux_tf, &aux_depth_debug)
                : std::nullopt;
            const bool aux_depth_available = aux_depth_sample.has_value();
            auto aux_depth_in_base = aux_depth_sample.has_value()
                ? std::optional<std::array<double, 3>>(transform_aux_to_base(aux_depth_sample->sampled_aux_point_3d, aux_tf))
                : std::nullopt;
            last_aux_tracked_ear_2d = std::nullopt;
            if (aux_body_ear_2d.has_value()) {
                last_aux_tracked_ear_2d = aux_body_ear_2d;
            } else if (aux_depth_sample.has_value()) {
                last_aux_tracked_ear_2d = (config.aux_view_mode == "color")
                    ? convert_3d_to_color_2d(aux_calibration, aux_depth_sample->sampled_aux_point_3d)
                    : std::optional<std::array<float, 2>>(aux_depth_sample->sampled_depth_2d);
            }

            std::optional<std::array<double, 3>> fused_ear;
            std::string fused_ear_source;
            if (base_ear_confident && aux_body_ear_in_base.has_value()) {
                if (effective_tracked_person_camera == "aux" && config.fusion_mode == "depth_only") {
                    fused_ear = fuse_depth_only_points(base_ear->position, *aux_body_ear_in_base);
                    fused_ear_source = "base_xy + aux_frontmost_z";
                } else if (config.fusion_mode == "full_3d") {
                    fused_ear = *aux_body_ear_in_base;
                    fused_ear_source = "aux_body_in_base_3d";
                } else {
                    fused_ear = fuse_depth_only_points(base_ear->position, *aux_body_ear_in_base);
                    fused_ear_source = "base_xy + aux_body_z";
                }
            } else if (base_ear_confident && aux_depth_available) {
                if (config.fusion_mode == "full_3d") {
                    fused_ear = *aux_depth_in_base;
                    fused_ear_source = "aux_depth_in_base_3d";
                } else {
                    fused_ear = fuse_depth_only_points(base_ear->position, *aux_depth_in_base);
                    fused_ear_source = "base_xy + aux_depth_z";
                }
            } else if (base_ear_confident) {
                fused_ear = base_ear->position;
                fused_ear_source = "base_only";
            } else if (effective_tracked_person_camera == "aux" && raw_aux_body_ear_in_base.has_value()) {
                fused_ear = *raw_aux_body_ear_in_base;
                fused_ear_source = "aux_frontmost_only";
            }

            const auto now = Clock::now();
            if (fused_ear.has_value()) {
                auto update_pal = [&](TrackingController *pal) {
                    pal->calculate_and_move(*fused_ear, now, false);
                };

                std::vector<std::thread> threads;
                threads.emplace_back(update_pal, &pal1);
                threads.emplace_back(update_pal, &pal2);
                threads.emplace_back(update_pal, &pal3);
                for (auto &th : threads) {
                    th.join();
                }

                last_fused_tracked_ear = fused_ear;
                last_fused_tracked_ear_2d = convert_3d_to_color_2d(base_calibration, *fused_ear);

                std::cout << "Fused " << config.tracked_ear << " ear [mm]: "
                          << (*fused_ear)[0] << ", "
                          << (*fused_ear)[1] << ", "
                          << (*fused_ear)[2]
                          << " | selection_camera=" << effective_tracked_person_camera
                          << " | base_z=" << (base_ear.has_value() ? base_ear->position[2] : 0.0)
                          << " | base_conf=" << (base_ear.has_value() ? joint_confidence_to_string(base_ear->confidence_level) : "MISSING")
                          << " aux_body=" << (aux_ear_confident ? "FOUND" : "MISSING")
                          << " aux_body_conf=" << (aux_ear.has_value() ? joint_confidence_to_string(aux_ear->confidence_level) : "MISSING")
                          << " aux_depth=" << (aux_depth_available ? "FOUND" : "MISSING")
                          << " aux_mode=" << (aux_camera_enabled ? wired_sync_mode_to_string(aux_config.wired_sync_mode) : "disabled")
                          << " source=" << fused_ear_source;
                if (aux_body_match_error_mm.has_value()) {
                    std::cout << " aux_body_match_err_mm=" << *aux_body_match_error_mm;
                }
                if (aux_body_rejected_by_match_gate) {
                    std::cout << " aux_body_rejected=match_gate";
                }
                if (aux_body_ear_in_base.has_value() &&
                    (fused_ear_source == "base_xy + aux_body_z" ||
                     fused_ear_source == "base_xy + aux_frontmost_z" ||
                     fused_ear_source == "aux_body_in_base_3d")) {
                    std::cout << " z_delta=" << ((*fused_ear)[2] - base_ear->position[2])
                              << " aux_body_z=" << (*aux_body_ear_in_base)[2];
                } else if (aux_depth_sample.has_value()) {
                    std::cout << " z_delta=" << ((*fused_ear)[2] - base_ear->position[2])
                              << " aux_match_err_mm=" << aux_depth_sample->spatial_error_mm;
                } else if (base_ear_confident) {
                    std::cout << " aux_reason=" << aux_depth_debug.failure_reason;
                    if (aux_depth_debug.predicted_depth_2d.has_value()) {
                        std::cout << " aux_pred_uv=(" << (*aux_depth_debug.predicted_depth_2d)[0] << ","
                                  << (*aux_depth_debug.predicted_depth_2d)[1] << ")";
                    }
                    if (aux_depth_debug.center_x >= 0 && aux_depth_debug.center_y >= 0) {
                        std::cout << " aux_center_uv=(" << aux_depth_debug.center_x << ","
                                  << aux_depth_debug.center_y << ")";
                    }
                    if (aux_depth_debug.depth_width > 0 && aux_depth_debug.depth_height > 0) {
                        std::cout << " aux_depth_size=" << aux_depth_debug.depth_width << "x"
                                  << aux_depth_debug.depth_height;
                    }
                    std::cout << " aux_nonzero_candidates=" << aux_depth_debug.nonzero_depth_candidates
                              << " aux_valid_candidates=" << aux_depth_debug.valid_3d_candidates;
                    if (std::isfinite(aux_depth_debug.best_spatial_error_mm)) {
                        std::cout << " aux_best_err_mm=" << aux_depth_debug.best_spatial_error_mm;
                    }
                }
                std::cout << '\n';
            }

            if (fusion_trace_writer.enabled()) {
                fusion_trace_writer.write_row(
                    ++fusion_trace_frame_index,
                    base_capture_generation,
                    aux_capture_generation,
                    base_depth_ts_for_trace,
                    aux_depth_ts_for_trace,
                    current_sync_phase_us,
                    current_sync_phase_error_us,
                    fused_ear_source,
                    base_ear,
                    aux_ear,
                    raw_aux_body_ear_in_base,
                    aux_body_ear_in_base.has_value(),
                    aux_body_match_error_mm,
                    aux_depth_in_base,
                    aux_depth_sample,
                    fused_ear,
                    aux_depth_debug);
            }

            if (base_color_image != nullptr && base_window_initialized && color_width > 0 && color_height > 0) {
                const uint8_t *src = k4a_image_get_buffer(base_color_image);
                const size_t image_size = static_cast<size_t>(color_width) * static_cast<size_t>(color_height) * 4;

                std::vector<uint8_t> display_buffer(image_size);
                std::memcpy(display_buffer.data(), src, image_size);

                if (last_fused_tracked_ear_2d.has_value()) {
                    const int cx = static_cast<int>((*last_fused_tracked_ear_2d)[0]);
                    const int cy = static_cast<int>((*last_fused_tracked_ear_2d)[1]);
                    draw_circle_bgra(display_buffer, color_width, color_height, cx, cy, 20);
                }

                base_image_window.show_bgra(display_buffer.data());
            }

            if (aux_window_initialized) {
                if (config.aux_view_mode == "color" &&
                    aux_color_image != nullptr &&
                    aux_color_width > 0 &&
                    aux_color_height > 0) {
                    const uint8_t *src = k4a_image_get_buffer(aux_color_image);
                    const size_t image_size = static_cast<size_t>(aux_color_width) * static_cast<size_t>(aux_color_height) * 4;
                    std::vector<uint8_t> display_buffer(image_size);
                    std::memcpy(display_buffer.data(), src, image_size);

                    if (last_aux_predicted_ear_2d.has_value()) {
                        const int cx = static_cast<int>((*last_aux_predicted_ear_2d)[0]);
                        const int cy = static_cast<int>((*last_aux_predicted_ear_2d)[1]);
                        draw_circle_bgra(display_buffer, aux_color_width, aux_color_height, cx, cy, 14, 0, 215, 255);
                    }
                    if (last_aux_tracked_ear_2d.has_value()) {
                        const int cx = static_cast<int>((*last_aux_tracked_ear_2d)[0]);
                        const int cy = static_cast<int>((*last_aux_tracked_ear_2d)[1]);
                        draw_circle_bgra(display_buffer, aux_color_width, aux_color_height, cx, cy, 20);
                    }

                    aux_image_window.show_bgra(display_buffer.data());
                } else if (aux_depth_image != nullptr && aux_depth_width > 0 && aux_depth_height > 0) {
                    std::vector<uint8_t> display_buffer = make_depth_bgra_buffer(aux_depth_image);

                    if (last_aux_predicted_ear_2d.has_value()) {
                        const int cx = static_cast<int>((*last_aux_predicted_ear_2d)[0]);
                        const int cy = static_cast<int>((*last_aux_predicted_ear_2d)[1]);
                        draw_circle_bgra(display_buffer, aux_depth_width, aux_depth_height, cx, cy, 14, 0, 215, 255);
                    }
                    if (last_aux_tracked_ear_2d.has_value()) {
                        const int cx = static_cast<int>((*last_aux_tracked_ear_2d)[0]);
                        const int cy = static_cast<int>((*last_aux_tracked_ear_2d)[1]);
                        draw_circle_bgra(display_buffer, aux_depth_width, aux_depth_height, cx, cy, 20);
                    }

                    aux_image_window.show_bgra(display_buffer.data());
                }
            }

            if (_kbhit()) {
                const int key = _getch();
                if (key == 'q') {
                    if (ptu1 != nullptr) {
                        ptu1->move(0, 0);
                    }
                    if (ptu2 != nullptr) {
                        ptu2->move(0, 0);
                    }
                    if (ptu3 != nullptr) {
                        ptu3->move(0, 0);
                    }
                    if (base_body_frame != nullptr) {
                        k4abt_frame_release(base_body_frame);
                        base_body_frame = nullptr;
                    }
                    if (aux_body_frame != nullptr) {
                        k4abt_frame_release(aux_body_frame);
                        aux_body_frame = nullptr;
                    }
                    if (base_color_image != nullptr) {
                        k4a_image_release(base_color_image);
                        base_color_image = nullptr;
                    }
                    if (aux_color_image != nullptr) {
                        k4a_image_release(aux_color_image);
                        aux_color_image = nullptr;
                    }
                    if (aux_depth_image != nullptr) {
                        k4a_image_release(aux_depth_image);
                        aux_depth_image = nullptr;
                    }
                    if (base_capture != nullptr) {
                        k4a_capture_release(base_capture);
                        base_capture = nullptr;
                    }
                    if (aux_capture != nullptr) {
                        k4a_capture_release(aux_capture);
                        aux_capture = nullptr;
                    }
                    break;
                }
                if (key == 'u' && last_fused_tracked_ear.has_value()) {
                    auto force_update = [&](TrackingController *pal) {
                        pal->calculate_and_move(*last_fused_tracked_ear, Clock::now(), true);
                    };

                    std::vector<std::thread> threads;
                    threads.emplace_back(force_update, &pal1);
                    threads.emplace_back(force_update, &pal2);
                    threads.emplace_back(force_update, &pal3);
                    for (auto &th : threads) {
                        th.join();
                    }
                    std::cout << "Manually updated\n";
                } else if (key == 'c') {
                    if (!image_capture_session.has_value()) {
                        warn_and_log("Image capture is disabled. Set enable_image_capture=true in the config.");
                    } else {
                        try {
                            const CaptureSaveResult save_result = save_capture_pair(
                                *image_capture_session,
                                base_color_image,
                                aux_color_image,
                                aux_depth_image);
                            info_and_log(format_capture_save_log_message(save_result));
                            const std::wstring status_text = make_capture_status_text(save_result);
                            base_image_window.show_status(status_text);
                            if (aux_window_initialized) {
                                aux_image_window.show_status(status_text);
                            }
                        } catch (const std::exception &capture_error) {
                            warn_and_log(std::string("Failed to save image capture: ") + capture_error.what());
                            base_image_window.show_status(L"Capture save failed");
                            if (aux_window_initialized) {
                                aux_image_window.show_status(L"Capture save failed");
                            }
                        }
                    }
                }
            }

            if (base_body_frame != nullptr) {
                k4abt_frame_release(base_body_frame);
            }
            if (aux_body_frame != nullptr) {
                k4abt_frame_release(aux_body_frame);
            }
            if (base_color_image != nullptr) {
                k4a_image_release(base_color_image);
            }
            if (aux_color_image != nullptr) {
                k4a_image_release(aux_color_image);
            }
            if (aux_depth_image != nullptr) {
                k4a_image_release(aux_depth_image);
            }
            if (base_capture != nullptr) {
                k4a_capture_release(base_capture);
            }
            if (aux_capture != nullptr) {
                k4a_capture_release(aux_capture);
            }

            if (pending_aux_subordinate_restart) {
                pending_aux_subordinate_restart = false;
                aux_capture_pump.stop();
                shutdown_and_destroy_tracker(&aux_tracker, "auxiliary");
                last_aux_capture_generation_enqueued = 0;
                last_aux_predicted_ear_2d.reset();
                last_aux_tracked_ear_2d.reset();
                show_aux_window_status(
                    L"Restarting aux Kinect in subordinate mode...",
                    kWindowRestartStatusDuration,
                    true);
                info_and_log(
                    "Attempting to restart auxiliary Kinect while keeping subordinate sync mode "
                    "after releasing in-flight captures.");
                if (restart_aux_camera_with_mode(
                        current_aux_device_index,
                        aux_device_serial_in_use,
                        &current_aux_device_index,
                        &aux_device,
                        aux_config,
                        &aux_calibration)) {
                    aux_camera_enabled = true;
                    create_aux_tracker_if_requested(true);
                    aux_capture_pump.start(aux_device, 100, "aux");
                    consecutive_aux_capture_failures = 0;
                    successful_dual_capture_count = 0;
                    consecutive_aux_alignment_failures = 0;
                    aux_subordinate_restart_attempted = false;
                    last_sync_sample_base_generation = 0;
                    last_sync_sample_aux_generation = 0;
                    sync_phase_baseline_sample_count = 0;
                    sync_phase_baseline_us.reset();
                    base_image_window.show_status(
                        L"Aux Kinect restarted; re-learning sync baseline.",
                        kWindowRestartStatusDuration);
                    show_aux_window_status(
                        L"Aux Kinect restarted; re-learning sync baseline.",
                        kWindowRestartStatusDuration,
                        true);
                    info_and_log("Aux Kinect restarted successfully in subordinate sync mode.");
                    info_and_log("Sync baseline reset after auxiliary restart.");
                } else {
                    warn_and_log("Failed to restart auxiliary Kinect in subordinate sync mode.");
                    if (config.allow_aux_unsynced_fallback && !aux_unsynced_fallback_attempted) {
                        aux_unsynced_fallback_attempted = true;
                        pending_aux_standalone_restart = true;
                        base_image_window.show_status(
                            L"Aux restart failed; falling back to standalone mode.",
                            kWindowRestartStatusDuration);
                        show_aux_window_status(
                            L"Aux restart failed; trying standalone mode...",
                            kWindowRestartStatusDuration,
                            true);
                        warn_and_log(
                            "Scheduling auxiliary Kinect restart in standalone mode after subordinate restart failure.");
                    } else {
                        warn_and_log("Disabling auxiliary Kinect and continuing with base Kinect only.");
                        aux_camera_enabled = false;
                        base_image_window.show_status(
                            L"Aux Kinect disabled; continuing in base-only mode.",
                            kWindowRestartStatusDuration);
                        show_aux_window_status(
                            L"Aux Kinect disabled; base-only mode.",
                            kWindowRestartStatusDuration,
                            true);
                    }
                }
                continue;
            }

            if (pending_aux_standalone_restart) {
                pending_aux_standalone_restart = false;
                const auto standalone_aux_config = make_aux_device_config(
                    base_config,
                    true,
                    config.subordinate_delay_off_master_usec,
                    config.aux_synchronized_images_only);
                aux_capture_pump.stop();
                shutdown_and_destroy_tracker(&aux_tracker, "auxiliary");
                last_aux_capture_generation_enqueued = 0;
                last_aux_predicted_ear_2d.reset();
                last_aux_tracked_ear_2d.reset();
                show_aux_window_status(
                    L"Restarting aux Kinect in standalone mode...",
                    kWindowRestartStatusDuration,
                    true);
                info_and_log(
                    "Attempting to restart auxiliary Kinect in standalone mode after releasing in-flight captures.");
                if (restart_aux_camera_with_mode(
                        current_aux_device_index,
                        aux_device_serial_in_use,
                        &current_aux_device_index,
                        &aux_device,
                        standalone_aux_config,
                        &aux_calibration)) {
                    aux_config = standalone_aux_config;
                    aux_is_unsynced = true;
                    aux_camera_enabled = true;
                    create_aux_tracker_if_requested(true);
                    aux_capture_pump.start(aux_device, 100, "aux");
                    consecutive_aux_capture_failures = 0;
                    successful_dual_capture_count = 0;
                    consecutive_aux_alignment_failures = 0;
                    last_sync_sample_base_generation = 0;
                    last_sync_sample_aux_generation = 0;
                    sync_phase_baseline_sample_count = 0;
                    sync_phase_baseline_us.reset();
                    base_image_window.show_status(
                        L"Aux Kinect restarted in standalone mode.",
                        kWindowRestartStatusDuration);
                    show_aux_window_status(
                        L"Aux Kinect restarted in standalone mode.",
                        kWindowRestartStatusDuration,
                        true);
                    info_and_log("Aux Kinect restarted successfully in standalone mode.");
                    info_and_log("Sync baseline reset after auxiliary restart.");
                } else {
                    warn_and_log("Failed to restart auxiliary Kinect in standalone mode.");
                    warn_and_log("Disabling auxiliary Kinect and continuing with base Kinect only.");
                    aux_camera_enabled = false;
                    aux_device = nullptr;
                    base_image_window.show_status(
                        L"Aux Kinect disabled; continuing in base-only mode.",
                        kWindowRestartStatusDuration);
                    show_aux_window_status(
                        L"Aux Kinect disabled; base-only mode.",
                        kWindowRestartStatusDuration,
                        true);
                }
                continue;
            }

        }

        shutdown_and_destroy_tracker(&base_tracker, "base");
        shutdown_and_destroy_tracker(&aux_tracker, "auxiliary");

        base_capture_pump.stop();
        aux_capture_pump.stop();

        if (base_device != nullptr) {
            k4a_device_stop_cameras(base_device);
            k4a_device_close(base_device);
        }
        if (aux_device != nullptr) {
            k4a_device_stop_cameras(aux_device);
            k4a_device_close(aux_device);
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << '\n';
        append_runtime_log(std::string("Error: ") + e.what());
        MessageBoxA(nullptr, e.what(), "track_test_cpp_2cam error", MB_OK | MB_ICONERROR);
        return 1;
    }

    return 0;
}
