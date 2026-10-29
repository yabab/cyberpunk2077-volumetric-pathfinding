cmake_minimum_required(VERSION 3.21)

project(vnav
  VERSION 0.1.0
  DESCRIPTION "Volumetric 3D pathfinding for flying agents in Cyberpunk 2077"
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(VNAV_BUILD_PLUGIN "Build the RED4ext plugin DLL (requires MSVC/Windows)." ON)
option(VNAV_BUILD_TESTS  "Build the standalone core test executable."            ON)
option(VNAV_BUILD_BAKE   "Build the offline nav baking tool."                    ON)

# OFF is the shipping state. Turn on for a bring-up or debugging run:
#   cmake -B build -DVNAV_LOGGING=ON
# The guard is `if constexpr`, so an off build pays nothing - not even for building the strings.
option(VNAV_LOGGING      "Emit plugin log output (off for shipping builds)."     OFF)

# The name RED4ext will load from red4ext/plugins/<name>/<name>.dll
set(VNAV_PLUGIN_NAME "PathfindingUpdate")

if(MSVC)
  add_compile_options(/W4 /permissive- /Zc:__cplusplus /MP /EHsc)
  add_compile_definitions(WIN32_LEAN_AND_MEAN NOMINMAX _CRT_SECURE_NO_WARNINGS)
  # Static CRT so the plugin never depends on a redistributable being present.
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# -----------------------------------------------------------------------------
# vnav_core - engine-agnostic. Must NOT include anything from RED4ext or the game.
# -----------------------------------------------------------------------------
add_library(vnav_core STATIC
  src/core/Chunk.cpp
  src/core/ChunkStore.cpp
  src/core/Sampler.cpp
  src/core/Planner.cpp
  src/core/Volume.cpp
  src/core/NavService.cpp
  src/core/BakedNav.cpp
)

target_include_directories(vnav_core
  PUBLIC  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

set_target_properties(vnav_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)
target_link_libraries(vnav_core PUBLIC Threads::Threads)

# -----------------------------------------------------------------------------
# The RED4ext plugin
# -----------------------------------------------------------------------------
if(VNAV_BUILD_PLUGIN)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/RED4ext.SDK/CMakeLists.txt")
    message(FATAL_ERROR
      "vendor/RED4ext.SDK is missing. Run:\n"
      "  git clone https://github.com/WopsS/RED4ext.SDK.git vendor/RED4ext.SDK")
  endif()

  add_subdirectory(vendor/RED4ext.SDK EXCLUDE_FROM_ALL)

  add_library(${VNAV_PLUGIN_NAME} SHARED
    src/game/Plugin.cpp
    src/game/Runtime.cpp
    src/game/RedWorldProbe.cpp
    src/game/ScriptBindings.cpp
    src/capi/CApi.cpp
  )

  target_include_directories(${VNAV_PLUGIN_NAME}
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )

  target_link_libraries(${VNAV_PLUGIN_NAME} PRIVATE vnav_core RED4ext::SDK)

  if(VNAV_LOGGING)
    target_compile_definitions(${VNAV_PLUGIN_NAME} PRIVATE VNAV_LOGGING=1)
    message(STATUS "vnav: plugin logging ENABLED (do not ship this build)")
  endif()

  set_target_properties(${VNAV_PLUGIN_NAME} PROPERTIES
    OUTPUT_NAME "${VNAV_PLUGIN_NAME}"
    PREFIX ""
  )

  # Stage straight into code/red4ext/plugins/<name>/ so myMods/deploy.ps1 can mirror it.
  set(VNAV_STAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/code/red4ext/plugins/${VNAV_PLUGIN_NAME}")
  add_custom_command(TARGET ${VNAV_PLUGIN_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${VNAV_STAGE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${VNAV_PLUGIN_NAME}>" "${VNAV_STAGE_DIR}/"
    COMMENT "Staging ${VNAV_PLUGIN_NAME}.dll into code/red4ext/plugins/"
  )
endif()

# -----------------------------------------------------------------------------
# vnav_bake - offline tool. Reads the .vnavgeo dump produced by WolvenKit's
# `navbake` command and voxelises it into a baked clearance field.
# -----------------------------------------------------------------------------
if(VNAV_BUILD_BAKE)
  add_executable(vnav_bake
    src/bake/Main.cpp
    src/bake/GeoFile.cpp
    src/bake/Voxelizer.cpp
  )
  target_include_directories(vnav_bake PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
  target_link_libraries(vnav_bake PRIVATE vnav_core)
endif()

# -----------------------------------------------------------------------------
# Standalone core tests - no game, no RED4ext. Runs on the command line.
# -----------------------------------------------------------------------------
if(VNAV_BUILD_TESTS)
  add_executable(vnav_tests tests/TestCore.cpp)
  target_include_directories(vnav_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
  target_link_libraries(vnav_tests PRIVATE vnav_core)

  enable_testing()
  add_test(NAME vnav_core_tests COMMAND vnav_tests)
endif()
