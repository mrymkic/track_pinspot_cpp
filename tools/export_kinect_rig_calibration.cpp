#include <k4a/k4a.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct DeviceInfo {
    uint32_t index = 0;
    std::string serial;
};

struct Options {
    fs::path output_path = fs::path("tools") / "checkerboard_rig_live.json";
    std::optional<uint32_t> base_index;
    std::optional<uint32_t> aux_index;
    std::string base_serial;
    std::string aux_serial;
    k4a_depth_mode_t depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    k4a_color_resolution_t color_resolution = K4A_COLOR_RESOLUTION_720P;
};

[[noreturn]] void fail(const std::string &message)
{
    throw std::runtime_error(message);
}

std::string json_escape(const std::string &value)
{
    std::ostringstream oss;
    for (const char ch : value) {
        switch (ch) {
        case '\\':
            oss << "\\\\";
            break;
        case '"':
            oss << "\\\"";
            break;
        case '\n':
            oss << "\\n";
            break;
        case '\r':
            oss << "\\r";
            break;
        case '\t':
            oss << "\\t";
            break;
        default:
            oss << ch;
            break;
        }
    }
    return oss.str();
}

std::string depth_mode_to_string(k4a_depth_mode_t mode)
{
    switch (mode) {
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
        return "nfov_2x2binned";
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
        return "nfov_unbinned";
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
        return "wfov_2x2binned";
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
        return "wfov_unbinned";
    case K4A_DEPTH_MODE_PASSIVE_IR:
        return "passive_ir";
    case K4A_DEPTH_MODE_OFF:
    default:
        return "off";
    }
}

std::string color_resolution_to_string(k4a_color_resolution_t resolution)
{
    switch (resolution) {
    case K4A_COLOR_RESOLUTION_720P:
        return "720p";
    case K4A_COLOR_RESOLUTION_1080P:
        return "1080p";
    case K4A_COLOR_RESOLUTION_1440P:
        return "1440p";
    case K4A_COLOR_RESOLUTION_1536P:
        return "1536p";
    case K4A_COLOR_RESOLUTION_2160P:
        return "2160p";
    case K4A_COLOR_RESOLUTION_3072P:
        return "3072p";
    case K4A_COLOR_RESOLUTION_OFF:
    default:
        return "off";
    }
}

k4a_depth_mode_t parse_depth_mode(const std::string &value)
{
    if (value == "nfov_2x2binned") {
        return K4A_DEPTH_MODE_NFOV_2X2BINNED;
    }
    if (value == "nfov_unbinned") {
        return K4A_DEPTH_MODE_NFOV_UNBINNED;
    }
    if (value == "wfov_2x2binned") {
        return K4A_DEPTH_MODE_WFOV_2X2BINNED;
    }
    if (value == "wfov_unbinned") {
        return K4A_DEPTH_MODE_WFOV_UNBINNED;
    }
    if (value == "passive_ir") {
        return K4A_DEPTH_MODE_PASSIVE_IR;
    }
    fail("Unsupported --depth-mode: " + value);
}

k4a_color_resolution_t parse_color_resolution(const std::string &value)
{
    if (value == "720p") {
        return K4A_COLOR_RESOLUTION_720P;
    }
    if (value == "1080p") {
        return K4A_COLOR_RESOLUTION_1080P;
    }
    if (value == "1440p") {
        return K4A_COLOR_RESOLUTION_1440P;
    }
    if (value == "1536p") {
        return K4A_COLOR_RESOLUTION_1536P;
    }
    if (value == "2160p") {
        return K4A_COLOR_RESOLUTION_2160P;
    }
    if (value == "3072p") {
        return K4A_COLOR_RESOLUTION_3072P;
    }
    fail("Unsupported --color-resolution: " + value);
}

uint32_t parse_u32(const std::string &value, const std::string &label)
{
    try {
        return static_cast<uint32_t>(std::stoul(value));
    } catch (const std::exception &) {
        fail("Invalid " + label + ": " + value);
    }
}

Options parse_args(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string &flag) -> std::string {
            if (i + 1 >= argc) {
                fail("Missing value for " + flag);
            }
            ++i;
            return argv[i];
        };

        if (arg == "--output") {
            options.output_path = require_value(arg);
        } else if (arg == "--base-index") {
            options.base_index = parse_u32(require_value(arg), arg);
        } else if (arg == "--aux-index") {
            options.aux_index = parse_u32(require_value(arg), arg);
        } else if (arg == "--base-serial") {
            options.base_serial = require_value(arg);
        } else if (arg == "--aux-serial") {
            options.aux_serial = require_value(arg);
        } else if (arg == "--depth-mode") {
            options.depth_mode = parse_depth_mode(require_value(arg));
        } else if (arg == "--color-resolution") {
            options.color_resolution = parse_color_resolution(require_value(arg));
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: export_kinect_rig_calibration [options]\n"
                << "  --output PATH              Output JSON path. Default: tools/checkerboard_rig_live.json\n"
                << "  --base-index N             Base Kinect device index override.\n"
                << "  --aux-index N              Aux Kinect device index override.\n"
                << "  --base-serial SERIAL       Base Kinect serial override.\n"
                << "  --aux-serial SERIAL        Aux Kinect serial override.\n"
                << "  --depth-mode MODE          nfov_2x2binned | nfov_unbinned | wfov_2x2binned | wfov_unbinned | passive_ir\n"
                << "  --color-resolution MODE    720p | 1080p | 1440p | 1536p | 2160p | 3072p\n";
            std::exit(0);
        } else {
            fail("Unknown argument: " + arg);
        }
    }
    return options;
}

std::string read_serial(k4a_device_t device)
{
    size_t buffer_size = 0;
    const k4a_buffer_result_t probe_result = k4a_device_get_serialnum(device, nullptr, &buffer_size);
    if (probe_result != K4A_BUFFER_RESULT_TOO_SMALL || buffer_size == 0) {
        return {};
    }

    std::string serial(buffer_size, '\0');
    if (k4a_device_get_serialnum(device, serial.data(), &buffer_size) != K4A_BUFFER_RESULT_SUCCEEDED) {
        return {};
    }
    while (!serial.empty() && serial.back() == '\0') {
        serial.pop_back();
    }
    return serial;
}

std::vector<DeviceInfo> enumerate_devices()
{
    const uint32_t installed_count = k4a_device_get_installed_count();
    std::vector<DeviceInfo> devices;
    for (uint32_t index = 0; index < installed_count; ++index) {
        k4a_device_t device = nullptr;
        if (k4a_device_open(index, &device) != K4A_RESULT_SUCCEEDED) {
            continue;
        }
        DeviceInfo info;
        info.index = index;
        info.serial = read_serial(device);
        devices.push_back(info);
        k4a_device_close(device);
    }
    return devices;
}

uint32_t resolve_device_index(
    const std::vector<DeviceInfo> &devices,
    const std::string &requested_serial,
    const std::optional<uint32_t> &requested_index,
    uint32_t fallback_index,
    const std::string &label)
{
    if (!requested_serial.empty()) {
        for (const auto &device : devices) {
            if (device.serial == requested_serial) {
                return device.index;
            }
        }
        fail("Could not find " + label + " serial: " + requested_serial);
    }

    if (requested_index.has_value()) {
        return *requested_index;
    }

    return fallback_index;
}

std::optional<std::string> find_serial_by_index(const std::vector<DeviceInfo> &devices, uint32_t index)
{
    for (const auto &device : devices) {
        if (device.index == index) {
            return device.serial;
        }
    }
    return std::nullopt;
}

k4a_calibration_t fetch_calibration(uint32_t device_index, k4a_depth_mode_t depth_mode, k4a_color_resolution_t color_resolution)
{
    k4a_device_t device = nullptr;
    if (k4a_device_open(device_index, &device) != K4A_RESULT_SUCCEEDED) {
        fail("Failed to open Azure Kinect device index " + std::to_string(device_index));
    }

    k4a_calibration_t calibration{};
    const k4a_result_t result = k4a_device_get_calibration(device, depth_mode, color_resolution, &calibration);
    k4a_device_close(device);
    if (result != K4A_RESULT_SUCCEEDED) {
        fail(
            "Failed to get calibration for device index " + std::to_string(device_index) +
            " with depth mode " + depth_mode_to_string(depth_mode) +
            " and color resolution " + color_resolution_to_string(color_resolution));
    }
    return calibration;
}

void write_indent(std::ostream &os, int indent)
{
    for (int i = 0; i < indent; ++i) {
        os.put(' ');
    }
}

void write_rotation_matrix(std::ostream &os, const float rotation[9], int indent)
{
    os << "[\n";
    for (int row = 0; row < 3; ++row) {
        write_indent(os, indent + 4);
        os << "[" << std::fixed << std::setprecision(8)
           << rotation[row * 3 + 0] << ", "
           << rotation[row * 3 + 1] << ", "
           << rotation[row * 3 + 2] << "]";
        if (row != 2) {
            os << ",";
        }
        os << "\n";
    }
    write_indent(os, indent);
    os << "]";
}

void write_translation_vector(std::ostream &os, const float translation[3])
{
    os << "[" << std::fixed << std::setprecision(8)
       << translation[0] << ", "
       << translation[1] << ", "
       << translation[2] << "]";
}

void write_camera_block(
    std::ostream &os,
    const std::string &name,
    const std::string &serial,
    const k4a_calibration_t &calibration,
    int indent)
{
    const auto &color_camera = calibration.color_camera_calibration;
    const auto &intrinsic = color_camera.intrinsics.parameters.param;
    const auto &color_to_depth = calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH];

    write_indent(os, indent);
    os << "\"" << name << "\": {\n";

    write_indent(os, indent + 4);
    os << "\"serial\": \"" << json_escape(serial) << "\",\n";

    write_indent(os, indent + 4);
    os << "\"color_camera_matrix\": [\n";
    write_indent(os, indent + 8);
    os << "[" << std::fixed << std::setprecision(8) << intrinsic.fx << ", 0.0, " << intrinsic.cx << "],\n";
    write_indent(os, indent + 8);
    os << "[0.0, " << intrinsic.fy << ", " << intrinsic.cy << "],\n";
    write_indent(os, indent + 8);
    os << "[0.0, 0.0, 1.0]\n";
    write_indent(os, indent + 4);
    os << "],\n";

    write_indent(os, indent + 4);
    os << "\"color_distortion_coeffs\": ["
       << std::fixed << std::setprecision(8)
       << intrinsic.k1 << ", "
       << intrinsic.k2 << ", "
       << intrinsic.p1 << ", "
       << intrinsic.p2 << ", "
       << intrinsic.k3 << ", "
       << intrinsic.k4 << ", "
       << intrinsic.k5 << ", "
       << intrinsic.k6 << "],\n";

    write_indent(os, indent + 4);
    os << "\"color_to_depth_rotation\": ";
    write_rotation_matrix(os, color_to_depth.rotation, indent + 4);
    os << ",\n";

    write_indent(os, indent + 4);
    os << "\"color_to_depth_translation_mm\": ";
    write_translation_vector(os, color_to_depth.translation);
    os << "\n";

    write_indent(os, indent);
    os << "}";
}

void write_output_json(
    const fs::path &output_path,
    const Options &options,
    const std::string &base_serial,
    const std::string &aux_serial,
    const k4a_calibration_t &base_calibration,
    const k4a_calibration_t &aux_calibration)
{
    fs::create_directories(output_path.parent_path());
    std::ofstream os(output_path, std::ios::binary);
    if (!os) {
        fail("Failed to open output path for writing: " + output_path.string());
    }

    os << "{\n";
    os << "    \"_note\": \"Generated from connected Azure Kinect devices for checkerboard calibration.\",\n";
    os << "    \"_note_modes\": \"Use the same color resolution and depth mode here as the checkerboard image capture run.\",\n";
    os << "    \"depth_mode\": \"" << depth_mode_to_string(options.depth_mode) << "\",\n";
    os << "    \"color_resolution\": \"" << color_resolution_to_string(options.color_resolution) << "\",\n";
    write_camera_block(os, "base", base_serial, base_calibration, 4);
    os << ",\n";
    write_camera_block(os, "aux", aux_serial, aux_calibration, 4);
    os << "\n}\n";
}

int main(int argc, char **argv)
{
    try {
        const Options options = parse_args(argc, argv);
        const auto devices = enumerate_devices();
        if (devices.size() < 2) {
            fail("At least two Azure Kinect devices must be discoverable to export a base/aux rig calibration JSON.");
        }

        std::cout << "Detected Azure Kinect devices:\n";
        for (const auto &device : devices) {
            std::cout << "  index " << device.index << " serial " << (device.serial.empty() ? "<unknown>" : device.serial) << "\n";
        }

        const uint32_t base_index =
            resolve_device_index(devices, options.base_serial, options.base_index, 0, "base");
        const uint32_t aux_index =
            resolve_device_index(devices, options.aux_serial, options.aux_index, 1, "aux");
        if (base_index == aux_index) {
            fail("Base and aux resolved to the same device index.");
        }

        const std::string base_serial =
            find_serial_by_index(devices, base_index).value_or(options.base_serial);
        const std::string aux_serial =
            find_serial_by_index(devices, aux_index).value_or(options.aux_serial);

        std::cout << "Using base index " << base_index << " serial " << (base_serial.empty() ? "<unknown>" : base_serial) << "\n";
        std::cout << "Using aux  index " << aux_index << " serial " << (aux_serial.empty() ? "<unknown>" : aux_serial) << "\n";
        std::cout << "Calibration mode: depth=" << depth_mode_to_string(options.depth_mode)
                  << " color=" << color_resolution_to_string(options.color_resolution) << "\n";

        const auto base_calibration = fetch_calibration(base_index, options.depth_mode, options.color_resolution);
        const auto aux_calibration = fetch_calibration(aux_index, options.depth_mode, options.color_resolution);

        const fs::path output_path = fs::absolute(options.output_path);
        write_output_json(output_path, options, base_serial, aux_serial, base_calibration, aux_calibration);
        std::cout << "Wrote rig calibration JSON: " << output_path.string() << "\n";
        return 0;
    } catch (const std::exception &exc) {
        std::cerr << exc.what() << "\n";
        return 1;
    }
}
