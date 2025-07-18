// Copyright 2025 Stefan Zellmann
// SPDX-License-Identifier: Apache-2.0

// std
#include <iostream>

// anari_cpp
#define ANARI_EXTENSION_UTILITY_IMPL
#include <anari/anari_cpp.hpp>
// anari-math
#include <anari/anari_cpp/ext/linalg.h>

// stb_image
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace anari::math;

// ========================================================
// generate our test scene (where the AMR to ANARI
// conversion happens
// ========================================================
anari::World generateScene(anari::Device device)
{
  // input cells for the bricks below
  // but we don't use them, they'd be the
  // input to the builder (that we don't
  // include here for simplicty...)
  int4 cells[9] = { // x,y,z: pos, w: level
    { 0, 0, 0, 0 },
    { 1, 0, 0, 0 },
    { 1, 1, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 1, 0, 1, 0 },
    { 1, 1, 1, 0 },
    { 0, 1, 1, 0 },
    { 2, 0, 0, 1 },
  };

  // data per cell:
  float data[9] = {
    // first block:
    0.f, 0.5f,
    0.f, 0.5f,
    0.f, 0.5f,
    0.f, 0.5f,
    // second block:
    1.f,
  };

  // Here's where the brick builder functions would be
  // invoked. We'll for now assume the output is on the
  // host, but ideally, we'd move to the device and stay
  // there from here on. That would require the ANARI
  // device (barney) to support CUDA device pointers

  // instead we just define the bricks ourselves..

  std::vector<int3> blockDims(2);
  blockDims[0] = int3(2, 2, 2);
  blockDims[1] = int3(1, 1, 1);

  std::vector<size_t> blockOffsets(2);
  blockOffsets[0] = 0;
  blockOffsets[1] = 8;

  std::vector<int3> blockStart(2);
  blockStart[0] = { 0, 0, 0};
  blockStart[1] = { 1, 0, 0};

  std::vector<int> blockLevel(2);
  blockLevel[0] = 0;
  blockLevel[1] = 1;

  // refinement ratio (per level, not per block!)
  std::vector<unsigned> refinementRatio(2);
  refinementRatio[0] = 2;
  refinementRatio[1] = 2;

  auto field = anari::newObject<anari::SpatialField>(device, "amr");

  anari::setParameterArray1D(device,
      field,
      "block.level",
      ANARI_INT32,
      blockLevel.data(),
      blockLevel.size());
  anari::setParameterArray1D(device,
      field,
      "refinementRatio",
      ANARI_UINT32,
      refinementRatio.data(),
      refinementRatio.size());

#if 0 // current spec as per the ANARI-Docs PR
  std::vector<anari::Array3D> blockData(2);
  for (size_t i = 0; i < blockData.size(); ++i) {
    blockData[i] = anari::newArray3D(device,
        &data[blockOffsets[i]],
        blockDims[i].x,
        blockDims[i].y,
        blockDims[i].z);
  }

  anari::setParameterArray1D(device,
      field,
      "block.start",
      ANARI_INT32_VEC3,
      blockStart.data(),
      blockStart.size());
  anari::setParameterArray1D(device,
      field,
      "block.data",
      ANARI_ARRAY1D,
      blockData.data(),
      blockData.size());

  for (auto a : blockData)
    anari::release(device, a);

#else // the change I'd like to see in the spec (arrays are flat):
  struct box3i { int3 lower, upper; };

  std::vector<box3i> blockBounds(blockDims.size());
  for (size_t i = 0; i < blockBounds.size(); ++i) {
    blockBounds[i].lower = blockStart[i];
    blockBounds[i].upper = blockStart[i] + blockDims[i] - int3(1);
  }

  anari::setParameterArray1D(device,
      field,
      "block.bounds",
      ANARI_INT32_BOX3,
      blockBounds.data(),
      blockBounds.size());
  anari::setParameterArray1D(device,
      field,
      "data",
      ANARI_FLOAT32,
      &data[0],
      sizeof(data)/sizeof(data[0]));
#endif

  anari::commitParameters(device, field);

  // Volume //

  auto volume = anari::newObject<anari::Volume>(device, "transferFunction1D");
  anari::setParameter(device, volume, "value", field);

  std::vector<anari::math::float3> colors;
  std::vector<float> opacities;

  colors.emplace_back(0.f, 0.f, 1.f);
  colors.emplace_back(0.f, 1.f, 0.f);
  colors.emplace_back(1.f, 0.f, 0.f);
  colors.emplace_back(1.f, 0.f, 0.f);

  opacities.emplace_back(0.5f);
  opacities.emplace_back(1.f);

  anari::setAndReleaseParameter(device,
      volume,
      "color",
      anari::newArray1D(device, colors.data(), colors.size()));
  anari::setAndReleaseParameter(device,
      volume,
      "opacity",
      anari::newArray1D(device, opacities.data(), opacities.size()));
  float2 voxelRange(0.f, 1.f);
  anariSetParameter(
      device, volume, "valueRange", ANARI_FLOAT32_BOX1, &voxelRange);

  anari::commitParameters(device, volume);

  // Create World //

  anari::World world = anari::newObject<anari::World>(device);
  anari::setAndReleaseParameter(
      device, world, "volume", anari::newArray1D(device, &volume));
  anari::release(device, volume);
  return world;
}

// ========================================================
// query anari extensions
// ========================================================
static bool deviceHasExtension(anari::Library library,
    const std::string &deviceSubtype,
    const std::string &extName)
{
  const char **extensions =
      anariGetDeviceExtensions(library, deviceSubtype.c_str());

  for (; *extensions; extensions++) {
    if (*extensions == extName)
      return true;
  }
  return false;
}

// ========================================================
// Log ANARI errors
// ========================================================
static void statusFunc(const void * /*userData*/,
    ANARIDevice /*device*/,
    ANARIObject source,
    ANARIDataType /*sourceType*/,
    ANARIStatusSeverity severity,
    ANARIStatusCode /*code*/,
    const char *message)
{
  if (severity == ANARI_SEVERITY_FATAL_ERROR) {
    fprintf(stderr, "[FATAL][%p] %s\n", source, message);
    std::exit(1);
  } else if (severity == ANARI_SEVERITY_ERROR) {
    fprintf(stderr, "[ERROR][%p] %s\n", source, message);
  } else if (severity == ANARI_SEVERITY_WARNING) {
    fprintf(stderr, "[WARN ][%p] %s\n", source, message);
  } else if (severity == ANARI_SEVERITY_PERFORMANCE_WARNING) {
    fprintf(stderr, "[PERF ][%p] %s\n", source, message);
  }
  // Ignore INFO/DEBUG messages
}

// ========================================================
// Function to render a given frame (renderer+world+cam)
//  and produce an output image
// ========================================================
static void render(
    anari::Device device, anari::Frame frame, const std::string &fileName)
{
  // Render frame and print out duration property //

  anari::render(device, frame);
  anari::wait(device, frame);

  float duration = 0.f;
  anari::getProperty(device, frame, "duration", duration, ANARI_NO_WAIT);

  printf("rendered frame in %fms\n", duration * 1000);

  stbi_flip_vertically_on_write(1);
  auto fb = anari::map<uint32_t>(device, frame, "channel.color");
  stbi_write_png(
      fileName.c_str(), fb.width, fb.height, 4, fb.data, 4 * fb.width);
  anari::unmap(device, frame, "channel.color");

  std::cout << "Output: " << fileName << '\n';
}

int main()
{
  // Setup ANARI device //

  auto library = anari::loadLibrary("environment", statusFunc);
  auto device = anari::newDevice(library, "default");

  anari::Extensions extensions =
      anari::extension::getInstanceExtensionStruct(device, device);

  // if (!extensions.ANARI_KHR_SPATIAL_FIELD_AMR) // not in the standard yet..
  //   printf("WARNING: device doesn't support ANARI_KHR_SPATIAL_FIELD_AMR\n");
  if (!extensions.ANARI_KHR_CAMERA_PERSPECTIVE)
    printf("WARNING: device doesn't support ANARI_KHR_CAMERA_PERSPECTIVE\n");
  if (!extensions.ANARI_KHR_LIGHT_DIRECTIONAL)
    printf("WARNING: device doesn't support ANARI_KHR_LIGHT_DIRECTIONAL\n");

  // device-specific extensions:
  bool hasSamplerVolumeExt =
      deviceHasExtension(library, "default", "ANARI_VSNRAY_SAMPLER_VOLUME");

  // Create world from a helper function //

  auto world = generateScene(device);

  // Add a directional light source //

  auto light = anari::newObject<anari::Light>(device, "directional");
  anari::setParameterArray1D(device, world, "light", &light, 1);
  anari::release(device, light);

  // Create renderer //

  auto renderer = anari::newObject<anari::Renderer>(device, "default");
  const float4 backgroundColor = {0.1f, 0.1f, 0.1f, 1.f};
  anari::setParameter(device, renderer, "background", backgroundColor);
  anari::setParameter(device, renderer, "pixelSamples", 32);
  anari::commitParameters(device, renderer);

  // Create frame (top-level object) //

  auto frame = anari::newObject<anari::Frame>(device);

  uint2 imageSize = {800, 800};
  anari::setParameter(device, frame, "size", imageSize);
  anari::setParameter(device, frame, "channel.color", ANARI_UFIXED8_RGBA_SRGB);

  anari::setParameter(device, frame, "world", world);
  anari::setParameter(device, frame, "renderer", renderer);

  // Create camera (in an interactive app we'd do that repeatedly) //

  auto camera = anari::newObject<anari::Camera>(device, "perspective");

  float3 eye(2, 1, 10);
  float3 dir(0, 0, -1);
  float3 up(0, 1, 0);
  float fovy = 0.7853f;
  float aspect = 1.f;

  anari::setParameter(device, camera, "position", eye);
  anari::setParameter(device, camera, "direction", dir);
  anari::setParameter(device, camera, "up", up);
  anari::setParameter(device, camera, "fovy", fovy);
  anari::setParameter(device, camera, "aspect", aspect);

  anari::commitParameters(device, camera);

  anari::setParameter(device, frame, "camera", camera);

  anari::commitParameters(device, frame);

  // render frame //

  render(device, frame, "anari-amr-sample.png");

  // Cleanup remaining ANARI objets //

  anari::release(device, camera);
  anari::release(device, renderer);
  anari::release(device, world);
  anari::release(device, frame);
  anari::release(device, device);

  anari::unloadLibrary(library);

  return 0;
}
