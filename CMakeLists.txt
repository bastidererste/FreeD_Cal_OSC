cmake_minimum_required(VERSION 3.22)
project(LinInterpolOscGui VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

set(JUCE_SOURCE_DIR "" CACHE PATH "Path to a local JUCE checkout")

if (JUCE_SOURCE_DIR)
  if (NOT EXISTS "${JUCE_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "JUCE_SOURCE_DIR does not contain CMakeLists.txt: ${JUCE_SOURCE_DIR}")
  endif()
  add_subdirectory("${JUCE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/juce")
elseif (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/JUCE/CMakeLists.txt")
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/JUCE" "${CMAKE_BINARY_DIR}/juce")
else()
  FetchContent_Declare(
    juce
    GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
    GIT_TAG 8.0.4
  )
  FetchContent_MakeAvailable(juce)
endif()

function(add_lin_gui_app target_name product_name source_file)
  juce_add_gui_app(${target_name}
    PRODUCT_NAME "${product_name}"
  )

  target_sources(${target_name}
    PRIVATE
      ${source_file}
  )

  target_compile_definitions(${target_name}
    PRIVATE
      JUCE_WEB_BROWSER=0
      JUCE_USE_CURL=0
      JUCE_VST3_CAN_REPLACE_VST2=0
  )

  target_link_libraries(${target_name}
    PRIVATE
      juce::juce_gui_extra
      juce::juce_osc
    PUBLIC
      juce::juce_recommended_config_flags
      juce::juce_recommended_lto_flags
      juce::juce_recommended_warning_flags
  )

  juce_generate_juce_header(${target_name})
endfunction()

add_lin_gui_app(LinInterpolOscGui "LinInterpol OSC Starter" src/Main.cpp)
add_lin_gui_app(LensEncoderOscSim "Lens Encoder OSC Sim" src/LensEncoderSimMain.cpp)

set(ALGLIB_SOURCE_DIR "" CACHE PATH "Path to ALGLIB Free Edition root directory")
if (ALGLIB_SOURCE_DIR)
  if (NOT EXISTS "${ALGLIB_SOURCE_DIR}/src/interpolation.h")
    message(FATAL_ERROR "ALGLIB_SOURCE_DIR does not look like ALGLIB Free Edition root: ${ALGLIB_SOURCE_DIR}")
  endif()

  file(GLOB ALGLIB_SOURCES "${ALGLIB_SOURCE_DIR}/src/*.cpp")
  if (ALGLIB_SOURCES STREQUAL "")
    message(FATAL_ERROR "No ALGLIB source files found in ${ALGLIB_SOURCE_DIR}/src")
  endif()

  target_sources(LinInterpolOscGui PRIVATE ${ALGLIB_SOURCES})
  target_include_directories(LinInterpolOscGui PRIVATE "${ALGLIB_SOURCE_DIR}/src")
  target_compile_definitions(LinInterpolOscGui PRIVATE LININTERPOL_HAVE_ALGLIB=1)
else()
  target_compile_definitions(LinInterpolOscGui PRIVATE LININTERPOL_HAVE_ALGLIB=0)
endif()
