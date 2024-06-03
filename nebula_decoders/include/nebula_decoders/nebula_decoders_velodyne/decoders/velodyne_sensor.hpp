#pragma once
#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_scan_decoder.hpp"

#include <cstdint>

namespace nebula
{
namespace drivers
{
class VelodyneSensor
{

protected:
  static double single_firing_s;
  static double offset_packet_time;
public:
  // VelodyneDecoder(): single_firing_s(0.0), offset_packet_time(0.0) {}
  // To ignore an empty data blocks which is created by only VLS128 dual return mode case
  /// @brief each VLP lidars packat structure in user manual. If you know details, see commens in each <vlp_list>.hpp file.
  virtual int getNumPaddingBlocks(bool /* dual_return */) { return 0; }

  // calculate and stack the firing timing for each laser timeing used in getAzimuthCorrected to calculate the corrected azimuth
  /// @brief each VLP lidar laser timing in user manual. If you know details, see commens in each <vlp_list>.hpp file.
  virtual bool fillAzimuthCache() { return false; }

  // calculate the corrected azimuth from each firing timing.
  /// @brief each VLP calculating sample code and formula in user manual. If you know details, see commens in each <vlp_list>.hpp file.
  virtual uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order) = 0;

  // initialize single_firing_s and offset_packet_time
  virtual void initializeSingleFiringS() = 0;
  virtual void initializeOffsetPacketTime() = 0;

  // get single_firing_s and offset_packet_time
  static double getSingleFiringS() { return single_firing_s; }
  static double getOffsetPacketTime() { return offset_packet_time; }
};
}  // namespace drivers
}  // namespace nebula