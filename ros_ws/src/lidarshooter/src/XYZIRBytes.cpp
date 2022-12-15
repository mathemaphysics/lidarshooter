#include "XYZIRBytes.hpp"

#include <cstdint>
#include <vector>

lidarshooter::XYZIRBytes::XYZIRBytes(float xPosIn, float yPosIn, float zPosIn, float intensityIn, int ringIn)
    : xPos(xPosIn),
      yPos(yPosIn),
      zPos(zPosIn),
      intensity(intensityIn),
      ring(ringIn)
{
}

lidarshooter::XYZIRBytes::XYZIRBytes(std::uint8_t *xBegin, std::uint8_t *yBegin, std::uint8_t *zBegin, std::uint8_t *intensityBegin, std::uint8_t *ringBegin)
    : xPos(xBegin),
      yPos(yBegin),
      zPos(zBegin),
      intensity(intensityBegin),
      ring(ringBegin)
{
}

void lidarshooter::XYZIRBytes::addToCloud(sensor_msgs::PointCloud2& msgIn)
{
  for (int j = 0; j < 4; ++j)
    msgIn.data.push_back(xPos.byteArray[j]);
  for (int j = 0; j < 4; ++j)
    msgIn.data.push_back(yPos.byteArray[j]);
  for (int j = 0; j < 4; ++j)
    msgIn.data.push_back(zPos.byteArray[j]);
  for (int j = 0; j < 4; ++j)
    msgIn.data.push_back(0);
  for (int j = 0; j < 4; ++j)
    msgIn.data.push_back(intensity.byteArray[j]);
  for (int j = 0; j < 4; ++j)
    msgIn.data.push_back(ring.byteArray[j]);
  for (int j = 0; j < 8; ++j)
    msgIn.data.push_back(0);
}

