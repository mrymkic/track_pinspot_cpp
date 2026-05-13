cmake_minimum_required(VERSION 3.15)
project(track_test_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P AND NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR
            "This project must be configured for x64/amd64. "
            "Open an x64 Developer Command Prompt, delete the build directory, "
            "and re-run CMake."
        )
    endif()
    if(CMAKE_GENERATOR_PLATFORM AND NOT CMAKE_GENERATOR_PLATFORM MATCHES "^[Xx]64$")
        message(FATAL_ERROR
            "Unsupported generator platform: ${CMAKE_GENERATOR_PLATFORM}. "
            "Use an x64 generator/platform."
        )
    endif()
endif()

set(K4A_SDK_DIR "" CACHE PATH "Path to the Azure Kinect SDK root directory")
set(K4ABT_SDK_DIR "" CACHE PATH "Path to the Azure Kinect Body Tracking SDK root directory")
set(K4ABT_ORT_DIR "" CACHE PATH "Path to the Azure Kinect Body Tracking ONNXRuntime package/root directory")
set(CUDA_TOOLKIT_DIR "" CACHE PATH "Optional path to the NVIDIA CUDA Toolkit root directory")

function(pick_existing_dir out_var)
    foreach(candidate ${ARGN})
        if(candidate AND IS_DIRECTORY "${candidate}")
            file(TO_CMAKE_PATH "${candidate}" normalized_candidate)
            set(${out_var} "${normalized_candidate}" PARENT_SCOPE)
            return()
        endif()
    endforeach()

    set(${out_var} "" PARENT_SCOPE)
endfunction()

file(GLOB K4A_PACKAGE_DIR_HINTS
    LIST_DIRECTORIES true
    CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/../*/pin_spot/packages/Microsoft.Azure.Kinect.Sensor.1.4.1"
    "${CMAKE_SOURCE_DIR}/../*/try_kinect/packages/Microsoft.Azure.Kinect.Sensor.1.4.1"
    "${CMAKE_SOURCE_DIR}/../../*/pin_spot/packages/Microsoft.Azure.Kinect.Sensor.1.4.1"
    "${CMAKE_SOURCE_DIR}/../../*/try_kinect/packages/Microsoft.Azure.Kinect.Sensor.1.4.1"
)

file(GLOB K4ABT_PACKAGE_DIR_HINTS
    LIST_DIRECTORIES true
    CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/../*/pin_spot/packages/Microsoft.Azure.Kinect.BodyTracking.1.1.2"
    "${CMAKE_SOURCE_DIR}/../*/try_kinect/packages/Microsoft.Azure.Kinect.BodyTracking.1.1.2"
    "${CMAKE_SOURCE_DIR}/../../*/pin_spot/packages/Microsoft.Azure.Kinect.BodyTracking.1.1.2"
    "${CMAKE_SOURCE_DIR}/../../*/try_kinect/packages/Microsoft.Azure.Kinect.BodyTracking.1.1.2"
)

file(GLOB K4ABT_ORT_PACKAGE_DIR_HINTS
    LIST_DIRECTORIES true
    CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/../*/pin_spot/packages/Microsoft.Azure.Kinect.BodyTracking.ONNXRuntime.1.10.0"
    "${CMAKE_SOURCE_DIR}/../*/try_kinect/packages/Microsoft.Azure.Kinect.BodyTracking.ONNXRuntime.1.10.0"
    "${CMAKE_SOURCE_DIR}/../../*/pin_spot/packages/Microsoft.Azure.Kinect.BodyTracking.ONNXRuntime.1.10.0"
    "${CMAKE_SOURCE_DIR}/../../*/try_kinect/packages/Microsoft.Azure.Kinect.BodyTracking.ONNXRuntime.1.10.0"
)

pick_existing_dir(K4A_SDK_DIR_RESOLVED
    "${K4A_SDK_DIR}"
    "$ENV{K4A_SDK_DIR}"
    "$ENV{AZURE_KINECT_SDK_DIR}"
    "C:/Program Files/Azure Kinect SDK v1.4.2"
    "C:/Program Files/Azure Kinect SDK v1.4.1"
    ${K4A_PACKAGE_DIR_HINTS}
)

pick_existing_dir(K4ABT_SDK_DIR_RESOLVED
    "${K4ABT_SDK_DIR}"
    "$ENV{K4ABT_SDK_DIR}"
    "$ENV{AZURE_KINECT_BODY_TRACKING_SDK_DIR}"
    "C:/Program Files/Azure Kinect Body Tracking SDK"
    "C:/Program Files/Azure Kinect Body Tracking SDK v1.1.2"
    ${K4ABT_PACKAGE_DIR_HINTS}
)

pick_existing_dir(K4ABT_ORT_DIR_RESOLVED
    "${K4ABT_ORT_DIR}"
    "$ENV{K4ABT_ORT_DIR}"
    ${K4ABT_ORT_PACKAGE_DIR_HINTS}
)

pick_existing_dir(CUDA_TOOLKIT_DIR_RESOLVED
    "${CUDA_TOOLKIT_DIR}"
    "$ENV{CUDA_PATH}"
    "$ENV{CUDA_TOOLKIT_ROOT_DIR}"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.5"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.4"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.3"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.2"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.1"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.0"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.8"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.7"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.6"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.5"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.4"
)

if(NOT K4A_SDK_DIR_RESOLVED)
    message(FATAL_ERROR
        "Azure Kinect SDK was not found. "
        "Set -DK4A_SDK_DIR=\"C:/Program Files/Azure Kinect SDK v1.4.2\" "
        "or define the K4A_SDK_DIR environment variable."
    )
endif()

if(NOT K4ABT_SDK_DIR_RESOLVED)
    message(FATAL_ERROR
        "Azure Kinect Body Tracking SDK or package was not found. "
        "Set -DK4ABT_SDK_DIR=\"C:/Program Files/Azure Kinect Body Tracking SDK\" "
        "or point it to Microsoft.Azure.Kinect.BodyTracking.*."
    )
endif()

if(NOT K4ABT_ORT_DIR_RESOLVED)
    message(FATAL_ERROR
        "Azure Kinect Body Tracking ONNXRuntime package was not found. "
        "Set -DK4ABT_ORT_DIR to Microsoft.Azure.Kinect.BodyTracking.ONNXRuntime.*."
    )
endif()

unset(K4A_INCLUDE_DIR CACHE)
unset(K4ABT_INCLUDE_DIR CACHE)
unset(K4A_LIB CACHE)
unset(K4ABT_LIB CACHE)
unset(K4A_DLL CACHE)
unset(K4A_DEPTHENGINE_DLL CACHE)
unset(K4ABT_DLL CACHE)
unset(K4ABT_ORT_DLL CACHE)
unset(K4ABT_ORT_DIRECTML_DLL CACHE)
unset(K4ABT_ORT_PROVIDERS_SHARED_DLL CACHE)
unset(K4ABT_ORT_PROVIDERS_CUDA_DLL CACHE)
unset(K4ABT_ORT_PROVIDERS_TENSORRT_DLL CACHE)
unset(K4ABT_MODEL CACHE)
unset(K4ABT_MODEL_LITE CACHE)
unset(K4ABT_CUDA_CUDART_DLL CACHE)
unset(K4ABT_CUDA_CUBLAS_DLL CACHE)
unset(K4ABT_CUDA_CUBLASLT_DLL CACHE)
unset(K4ABT_CUDA_CUDNN_DLL CACHE)
unset(K4ABT_CUDA_CUDNN_CNN_INFER_DLL CACHE)
unset(K4ABT_CUDA_CUDNN_OPS_INFER_DLL CACHE)
unset(K4ABT_CUDA_CUFFT_DLL CACHE)
unset(K4ABT_CUDA_NVRTC_DLL CACHE)
unset(K4ABT_CUDA_NVRTC_BUILTINS_DLL CACHE)

find_path(K4A_INCLUDE_DIR
    NAMES k4a/k4a.h
    HINTS "${K4A_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/include" "build/native/include"
    NO_DEFAULT_PATH
)
find_path(K4ABT_INCLUDE_DIR
    NAMES k4abt.h
    HINTS "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/include" "build/native/include"
    NO_DEFAULT_PATH
)
find_library(K4A_LIB
    NAMES k4a.lib
    HINTS "${K4A_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/lib" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_library(K4ABT_LIB
    NAMES k4abt.lib
    HINTS "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/lib" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_file(K4A_DLL
    NAMES k4a.dll
    HINTS "${K4A_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_file(K4A_DEPTHENGINE_DLL
    NAMES depthengine_2_0.dll
    HINTS "${K4A_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_file(K4ABT_DLL
    NAMES k4abt.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_file(K4ABT_ORT_DLL
    NAMES onnxruntime.dll
    HINTS "${K4ABT_ORT_DIR_RESOLVED}" "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_file(K4ABT_ORT_DIRECTML_DLL
    NAMES directml.dll
    HINTS "${K4ABT_ORT_DIR_RESOLVED}" "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_file(K4ABT_ORT_PROVIDERS_SHARED_DLL
    NAMES onnxruntime_providers_shared.dll
    HINTS "${K4ABT_ORT_DIR_RESOLVED}" "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_file(K4ABT_ORT_PROVIDERS_CUDA_DLL
    NAMES onnxruntime_providers_cuda.dll
    HINTS "${K4ABT_ORT_DIR_RESOLVED}" "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_file(K4ABT_ORT_PROVIDERS_TENSORRT_DLL
    NAMES onnxruntime_providers_tensorrt.dll
    HINTS "${K4ABT_ORT_DIR_RESOLVED}" "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "lib/native/amd64/release"
    NO_DEFAULT_PATH
)
find_file(K4ABT_MODEL
    NAMES dnn_model_2_0_op11.onnx
    HINTS "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "content"
    NO_DEFAULT_PATH
)
find_file(K4ABT_MODEL_LITE
    NAMES dnn_model_2_0_lite_op11.onnx
    HINTS "${K4ABT_SDK_DIR_RESOLVED}"
    PATH_SUFFIXES "sdk/windows-desktop/amd64/release/bin" "content"
    NO_DEFAULT_PATH
)
find_file(K4ABT_CUDA_CUDART_DLL
    NAMES cudart64_110.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}" "${CUDA_TOOLKIT_DIR_RESOLVED}"
    PATH_SUFFIXES "tools" "sdk/windows-desktop/amd64/release/bin" "bin"
    NO_DEFAULT_PATH
)
find_file(K4ABT_CUDA_CUBLAS_DLL
    NAMES cublas64_11.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}" "${CUDA_TOOLKIT_DIR_RESOLVED}"
    PATH_SUFFIXES "tools" "sdk/windows-desktop/amd64/release/bin" "bin"
    NO_DEFAULT_PATH
)
find_file(K4ABT_CUDA_CUBLASLT_DLL
    NAMES cublasLt64_11.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}" "${CUDA_TOOLKIT_DIR_RESOLVED}"
    PATH_SUFFIXES "tools" "sdk/windows-desktop/amd64/release/bin" "bin"
    NO_DEFAULT_PATH
)
find_file(K4ABT_CUDA_CUDNN_DLL
    NAMES cudnn64_8.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}" "${CUDA_TOOLKIT_DIR_RESOLVED}"
    PATH_SUFFIXES "tools" "sdk/windows-desktop/amd64/release/bin" "bin"
    NO_DEFAULT_PATH
)
find_file(K4ABT_CUDA_CUDNN_CNN_INFER_DLL
    NAMES cudnn_cnn_infer64_8.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}" "${CUDA_TOOLKIT_DIR_RESOLVED}"
    PATH_SUFFIXES "tools" "sdk/windows-desktop/amd64/release/bin" "bin"
    NO_DEFAULT_PATH
)
find_file(K4ABT_CUDA_CUDNN_OPS_INFER_DLL
    NAMES cudnn_ops_infer64_8.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}" "${CUDA_TOOLKIT_DIR_RESOLVED}"
    PATH_SUFFIXES "tools" "sdk/windows-desktop/amd64/release/bin" "bin"
    NO_DEFAULT_PATH
)
find_file(K4ABT_CUDA_CUFFT_DLL
    NAMES cufft64_10.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}" "${CUDA_TOOLKIT_DIR_RESOLVED}"
    PATH_SUFFIXES "tools" "sdk/windows-desktop/amd64/release/bin" "bin"
    NO_DEFAULT_PATH
)
find_file(K4ABT_CUDA_NVRTC_DLL
    NAMES nvrtc64_112_0.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}" "${CUDA_TOOLKIT_DIR_RESOLVED}"
    PATH_SUFFIXES "tools" "sdk/windows-desktop/amd64/release/bin" "bin"
    NO_DEFAULT_PATH
)
find_file(K4ABT_CUDA_NVRTC_BUILTINS_DLL
    NAMES nvrtc-builtins64_114.dll
    HINTS "${K4ABT_SDK_DIR_RESOLVED}" "${CUDA_TOOLKIT_DIR_RESOLVED}"
    PATH_SUFFIXES "tools" "sdk/windows-desktop/amd64/release/bin" "bin"
    NO_DEFAULT_PATH
)

if(NOT K4A_INCLUDE_DIR OR NOT K4A_LIB OR NOT K4A_DLL OR NOT K4A_DEPTHENGINE_DLL)
    message(FATAL_ERROR
        "Azure Kinect SDK files are incomplete under ${K4A_SDK_DIR_RESOLVED}. "
        "Check that sdk/include, sdk/.../lib/k4a.lib, and runtime DLLs exist."
    )
endif()

if(NOT K4ABT_INCLUDE_DIR OR NOT K4ABT_LIB OR NOT K4ABT_DLL OR NOT K4ABT_ORT_DLL OR NOT K4ABT_MODEL)
    message(FATAL_ERROR
        "Azure Kinect Body Tracking files are incomplete under ${K4ABT_SDK_DIR_RESOLVED}. "
        "Check headers, k4abt.lib, k4abt.dll, the ONNX model, and the ONNXRuntime package."
    )
endif()

message(STATUS "Using Azure Kinect SDK: ${K4A_SDK_DIR_RESOLVED}")
message(STATUS "Using Azure Kinect Body Tracking SDK: ${K4ABT_SDK_DIR_RESOLVED}")
message(STATUS "Using Azure Kinect Body Tracking ONNXRuntime: ${K4ABT_ORT_DIR_RESOLVED}")
if(CUDA_TOOLKIT_DIR_RESOLVED)
    message(STATUS "Using CUDA Toolkit: ${CUDA_TOOLKIT_DIR_RESOLVED}")
endif()

set(K4ABT_CUDA_RUNTIME_MISSING "")
foreach(required_pair
    "K4ABT_CUDA_CUDART_DLL|cudart64_110.dll"
    "K4ABT_CUDA_CUBLAS_DLL|cublas64_11.dll"
    "K4ABT_CUDA_CUBLASLT_DLL|cublasLt64_11.dll"
    "K4ABT_CUDA_CUDNN_DLL|cudnn64_8.dll"
    "K4ABT_CUDA_CUDNN_CNN_INFER_DLL|cudnn_cnn_infer64_8.dll"
    "K4ABT_CUDA_CUDNN_OPS_INFER_DLL|cudnn_ops_infer64_8.dll"
    "K4ABT_CUDA_CUFFT_DLL|cufft64_10.dll"
    "K4ABT_CUDA_NVRTC_DLL|nvrtc64_112_0.dll"
    "K4ABT_CUDA_NVRTC_BUILTINS_DLL|nvrtc-builtins64_114.dll"
)
    string(REPLACE "|" ";" required_list "${required_pair}")
    list(GET required_list 0 required_var)
    list(GET required_list 1 required_name)
    if("${${required_var}}" STREQUAL "")
        list(APPEND K4ABT_CUDA_RUNTIME_MISSING "${required_name}")
    endif()
endforeach()

if(K4ABT_ORT_PROVIDERS_CUDA_DLL AND K4ABT_CUDA_RUNTIME_MISSING)
    message(WARNING
        "CUDA body-tracking runtime dependencies were not fully found. "
        "gpu_cuda may fail until the missing DLLs are installed or reachable via PATH. "
        "Missing: ${K4ABT_CUDA_RUNTIME_MISSING}"
    )
endif()

set(RUNTIME_STAGE_DIR "${CMAKE_BINARY_DIR}/runtime_stage")
file(MAKE_DIRECTORY "${RUNTIME_STAGE_DIR}")

function(stage_runtime_file out_var source_path staged_name)
    if(NOT source_path)
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()

    set(staged_path "${RUNTIME_STAGE_DIR}/${staged_name}")
    configure_file("${source_path}" "${staged_path}" COPYONLY)
    set(${out_var} "${staged_path}" PARENT_SCOPE)
endfunction()

stage_runtime_file(K4ABT_ORT_DLL_STAGED "${K4ABT_ORT_DLL}" "onnxruntime.dll")
stage_runtime_file(K4ABT_ORT_DIRECTML_DLL_STAGED "${K4ABT_ORT_DIRECTML_DLL}" "directml.dll")
stage_runtime_file(K4ABT_ORT_PROVIDERS_SHARED_DLL_STAGED "${K4ABT_ORT_PROVIDERS_SHARED_DLL}" "onnxruntime_providers_shared.dll")
stage_runtime_file(K4ABT_ORT_PROVIDERS_CUDA_DLL_STAGED "${K4ABT_ORT_PROVIDERS_CUDA_DLL}" "onnxruntime_providers_cuda.dll")
stage_runtime_file(K4ABT_ORT_PROVIDERS_TENSORRT_DLL_STAGED "${K4ABT_ORT_PROVIDERS_TENSORRT_DLL}" "onnxruntime_providers_tensorrt.dll")
stage_runtime_file(K4ABT_CUDA_CUDART_DLL_STAGED "${K4ABT_CUDA_CUDART_DLL}" "cudart64_110.dll")
stage_runtime_file(K4ABT_CUDA_CUBLAS_DLL_STAGED "${K4ABT_CUDA_CUBLAS_DLL}" "cublas64_11.dll")
stage_runtime_file(K4ABT_CUDA_CUBLASLT_DLL_STAGED "${K4ABT_CUDA_CUBLASLT_DLL}" "cublasLt64_11.dll")
stage_runtime_file(K4ABT_CUDA_CUDNN_DLL_STAGED "${K4ABT_CUDA_CUDNN_DLL}" "cudnn64_8.dll")
stage_runtime_file(K4ABT_CUDA_CUDNN_CNN_INFER_DLL_STAGED "${K4ABT_CUDA_CUDNN_CNN_INFER_DLL}" "cudnn_cnn_infer64_8.dll")
stage_runtime_file(K4ABT_CUDA_CUDNN_OPS_INFER_DLL_STAGED "${K4ABT_CUDA_CUDNN_OPS_INFER_DLL}" "cudnn_ops_infer64_8.dll")
stage_runtime_file(K4ABT_CUDA_CUFFT_DLL_STAGED "${K4ABT_CUDA_CUFFT_DLL}" "cufft64_10.dll")
stage_runtime_file(K4ABT_CUDA_NVRTC_DLL_STAGED "${K4ABT_CUDA_NVRTC_DLL}" "nvrtc64_112_0.dll")
stage_runtime_file(K4ABT_CUDA_NVRTC_BUILTINS_DLL_STAGED "${K4ABT_CUDA_NVRTC_BUILTINS_DLL}" "nvrtc-builtins64_114.dll")

function(add_track_target target_name source_name config_name)
    add_executable(${target_name}
        "${CMAKE_SOURCE_DIR}/${source_name}"
    )

    target_include_directories(${target_name} PRIVATE
        "${K4A_INCLUDE_DIR}"
        "${K4ABT_INCLUDE_DIR}"
    )

    target_link_libraries(${target_name} PRIVATE
        "${K4A_LIB}"
        "${K4ABT_LIB}"
        dxgi
        setupapi
    )

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${K4A_DLL}"
            $<TARGET_FILE_DIR:${target_name}>/k4a.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${K4A_DEPTHENGINE_DLL}"
            $<TARGET_FILE_DIR:${target_name}>/depthengine_2_0.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${K4ABT_DLL}"
            $<TARGET_FILE_DIR:${target_name}>/k4abt.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${K4ABT_ORT_DLL_STAGED}"
            $<TARGET_FILE_DIR:${target_name}>/onnxruntime.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${K4ABT_MODEL}"
            $<TARGET_FILE_DIR:${target_name}>/dnn_model_2_0_op11.onnx
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/${config_name}"
            $<TARGET_FILE_DIR:${target_name}>/${config_name}
        COMMENT "Copying runtime dependencies for ${target_name}..."
    )

    if(config_name STREQUAL "track_config_2.json")
        foreach(extra_config
            track_config_2_bt_cpu.json
            track_config_2_bt_cuda.json
            track_config_2_bt_cuda_dual.json
            track_config_2_bt_cuda_lite.json
            track_config_2_bt_cuda_lite_dual.json
            track_config_2_bt_cuda_lite_dual_eval.json
            track_config_2_capture_images.json
            track_config_2_capture_only.json
        )
            add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${CMAKE_SOURCE_DIR}/${extra_config}"
                    $<TARGET_FILE_DIR:${target_name}>/${extra_config}
            )
        endforeach()
    endif()

    if(K4ABT_MODEL_LITE)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${K4ABT_MODEL_LITE}"
                $<TARGET_FILE_DIR:${target_name}>/dnn_model_2_0_lite_op11.onnx
        )
    endif()

    if(K4ABT_ORT_DIRECTML_DLL_STAGED)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${K4ABT_ORT_DIRECTML_DLL_STAGED}"
                $<TARGET_FILE_DIR:${target_name}>/directml.dll
        )
    endif()

    if(K4ABT_ORT_PROVIDERS_SHARED_DLL_STAGED)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${K4ABT_ORT_PROVIDERS_SHARED_DLL_STAGED}"
                $<TARGET_FILE_DIR:${target_name}>/onnxruntime_providers_shared.dll
        )
    endif()

    if(K4ABT_ORT_PROVIDERS_CUDA_DLL_STAGED)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${K4ABT_ORT_PROVIDERS_CUDA_DLL_STAGED}"
                $<TARGET_FILE_DIR:${target_name}>/onnxruntime_providers_cuda.dll
        )
    endif()

    foreach(optional_cuda_runtime
        "K4ABT_CUDA_CUDART_DLL_STAGED|cudart64_110.dll"
        "K4ABT_CUDA_CUBLAS_DLL_STAGED|cublas64_11.dll"
        "K4ABT_CUDA_CUBLASLT_DLL_STAGED|cublasLt64_11.dll"
        "K4ABT_CUDA_CUDNN_DLL_STAGED|cudnn64_8.dll"
        "K4ABT_CUDA_CUDNN_CNN_INFER_DLL_STAGED|cudnn_cnn_infer64_8.dll"
        "K4ABT_CUDA_CUDNN_OPS_INFER_DLL_STAGED|cudnn_ops_infer64_8.dll"
        "K4ABT_CUDA_CUFFT_DLL_STAGED|cufft64_10.dll"
        "K4ABT_CUDA_NVRTC_DLL_STAGED|nvrtc64_112_0.dll"
        "K4ABT_CUDA_NVRTC_BUILTINS_DLL_STAGED|nvrtc-builtins64_114.dll"
    )
        string(REPLACE "|" ";" optional_cuda_runtime_list "${optional_cuda_runtime}")
        list(GET optional_cuda_runtime_list 0 optional_cuda_runtime_var)
        list(GET optional_cuda_runtime_list 1 optional_cuda_runtime_name)
        if(NOT "${${optional_cuda_runtime_var}}" STREQUAL "")
            add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${${optional_cuda_runtime_var}}"
                    $<TARGET_FILE_DIR:${target_name}>/${optional_cuda_runtime_name}
            )
        endif()
    endforeach()

    if(K4ABT_ORT_PROVIDERS_TENSORRT_DLL_STAGED)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${K4ABT_ORT_PROVIDERS_TENSORRT_DLL_STAGED}"
                $<TARGET_FILE_DIR:${target_name}>/onnxruntime_providers_tensorrt.dll
        )
    endif()
endfunction()

add_track_target(track_test_cpp "track_test.cpp" "track_config.json")
add_track_target(track_test_cpp_2cam "track_test_2.cpp" "track_config_2.json")
