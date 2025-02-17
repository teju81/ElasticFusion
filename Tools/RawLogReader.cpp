/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at
 * <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "RawLogReader.h"

RawLogReader::RawLogReader(std::string file, bool flipColors) : LogReader(file, flipColors) {
  assert(pangolin::FileExists(file.c_str()));

  fp = fopen(file.c_str(), "rb"); // Binary log file passed to the program with flag -l

  currentFrame = 0;

  auto tmp = fread(&numFrames, sizeof(int32_t), 1, fp);
  assert(tmp);
  std::cout<<"num of frames: " << numFrames << " num of pixels: " << numPixels << std::endl;

  depthReadBuffer = new uint8_t[numPixels * 2];
  imageReadBuffer = new uint8_t[numPixels * 3];
  decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
  decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];
}

RawLogReader::~RawLogReader() {
  delete[] depthReadBuffer;
  delete[] imageReadBuffer;
  delete[] decompressionBufferDepth;
  delete[] decompressionBufferImage;

  fclose(fp);
}

void RawLogReader::getBack() {
  assert(filePointers.size() > 0);

  fseek(fp, filePointers.top(), SEEK_SET);

  filePointers.pop();

  getCore();
}

void RawLogReader::getNext() {
  std::cout << "In RawLogReader"<< std::endl;

  filePointers.push(ftell(fp));
  std::cout << "finished pushing file pointer"<< std::endl;

  getCore();
}

void RawLogReader::getCore() {
  std::cout << "Inside core reader"<< std::endl;
  auto tmp = fread(&timestamp, sizeof(int64_t), 1, fp);
  assert(tmp);

  tmp = fread(&depthSize, sizeof(int32_t), 1, fp);
  assert(tmp);
  std::cout << "DepthSize: "<< depthSize << std::endl;
  std::cout << "DepthSize assert done"<< std::endl;


  tmp = fread(&imageSize, sizeof(int32_t), 1, fp);
  assert(tmp);
  std::cout << "imageSize: "<< imageSize << std::endl;
  std::cout << "imageSize assert done"<< std::endl;


  std::cout << "Reading depth buffer...." << std::endl;

  depthReadBuffer = static_cast<uint8_t*>(std::malloc(depthSize));
  if (depthReadBuffer == nullptr) {
      std::cerr << "Error: Depth Memory allocation failed." << std::endl;
      // Handle the error appropriately, e.g., return or throw an exception
  }

  tmp = fread(depthReadBuffer, depthSize, 1, fp);
  assert(tmp);
  std::cout << "Depth buffer assert done"<< std::endl;


  imageReadBuffer = static_cast<uint8_t*>(std::malloc(imageSize));
  if (imageReadBuffer == nullptr) {
      std::cerr << "Error: Image Memory allocation failed." << std::endl;
      // Handle the error appropriately, e.g., return or throw an exception
  }


  if (imageSize > 0) {
    tmp = fread(imageReadBuffer, imageSize, 1, fp);
    assert(tmp);
    std::cout << "Image buffer assert done"<< std::endl;
  }
  std::cout << "Passed Assert statements"<< std::endl;

  // if (depthSize == numPixels * 2) {
  //   std::cout << "Depth condition 1"<< std::endl;
  //   memcpy(&decompressionBufferDepth[0], depthReadBuffer, numPixels * 2);
  // } else {
  //   std::cout << "Depth condition 2: " << depthSize << " " << numPixels << std::endl;
  //   unsigned long decompLength = numPixels * 2;
  //   uncompress(
  //       &decompressionBufferDepth[0],
  //       (unsigned long*)&decompLength,
  //       (const Bytef*)depthReadBuffer,
  //       depthSize);
  // }


  // if (depthSize == numPixels * 2) {
  //   std::cout << "Depth condition 1: " << depthSize << " " << numPixels << std::endl;
  //   memcpy(&decompressionBufferDepth[0], depthReadBuffer, numPixels * 2);
  // }

  // if (imageSize == numPixels * 3) {
  //   std::cout << "Image condition 1: " << imageSize << " " << numPixels << std::endl;
  //   memcpy(&decompressionBufferImage[0], imageReadBuffer, numPixels * 3);
  // } else if (imageSize > 0) {
  //   std::cout << "Image condition 2: " << imageSize << " " << numPixels << std::endl;
  //   //jpeg.readData(imageReadBuffer, imageSize, (uint8_t*)&decompressionBufferImage[0]);

  //   unsigned long decompLength = numPixels * 2;
  //   uncompress(
  //       &decompressionBufferImage[0],
  //       (unsigned long*)&decompLength,
  //       (const Bytef*)imageReadBuffer,
  //       imageSize);

  // } else {
  //   memset(&decompressionBufferImage[0], 0, numPixels * 3);
  // }

  std::cout << "Depth condition 1: " << depthSize << " " << numPixels << std::endl;
  std::cout << "Image condition 2: " << imageSize << " " << numPixels << std::endl;

  memcpy(&decompressionBufferDepth[0], depthReadBuffer, numPixels * 2);
  memcpy(&decompressionBufferImage[0], imageReadBuffer, numPixels * 3);


  depth = (uint16_t*)decompressionBufferDepth;
  rgb = (uint8_t*)&decompressionBufferImage[0];

  if (flipColors) {
    for (int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3) {
      std::swap(rgb[i + 0], rgb[i + 2]);
    }
  }

  currentFrame++;
  std::cout << "Finished core reader"<< std::endl;
}

void RawLogReader::fastForward(int frame) {
  while (currentFrame < frame && hasMore()) {
    filePointers.push(ftell(fp));

    auto tmp = fread(&timestamp, sizeof(int64_t), 1, fp);
    assert(tmp);

    tmp = fread(&depthSize, sizeof(int32_t), 1, fp);
    assert(tmp);
    tmp = fread(&imageSize, sizeof(int32_t), 1, fp);
    assert(tmp);

    tmp = fread(depthReadBuffer, depthSize, 1, fp);
    assert(tmp);

    if (imageSize > 0) {
      tmp = fread(imageReadBuffer, imageSize, 1, fp);
      assert(tmp);
    }

    currentFrame++;
  }
}

int RawLogReader::getNumFrames() {
  return numFrames;
}

bool RawLogReader::hasMore() {
  return currentFrame + 1 < numFrames;
}

void RawLogReader::rewind() {
  if (filePointers.size() != 0) {
    std::stack<int> empty;
    std::swap(empty, filePointers);
  }

  fclose(fp);
  fp = fopen(file.c_str(), "rb");

  auto tmp = fread(&numFrames, sizeof(int32_t), 1, fp);
  assert(tmp);

  currentFrame = 0;
}

bool RawLogReader::rewound() {
  return filePointers.size() == 0;
}

const std::string RawLogReader::getFile() {
  return file;
}

void RawLogReader::setAuto(bool value) {}
