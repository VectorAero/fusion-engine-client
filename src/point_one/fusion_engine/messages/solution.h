/**************************************************************************/ /**
 * @brief Platform position/attitude solution messages.
 * @file
 ******************************************************************************/

#pragma once

#include "point_one/fusion_engine/messages/defs.h"

namespace point_one {
namespace fusion_engine {
namespace messages {

// Enforce 4-byte alignment and packing of all data structures and values so
// that floating point values are aligned on platforms that require it.
#pragma pack(push, 4)

/**
 * @brief Platform pose solution: position, velocity, attitude (@ref
 *        MessageType::POSE).
 * @ingroup messages
 *
 * @note
 * All data is timestamped using the Point One Time, which is a monotonic
 * timestamp referenced to the start of the device. Corresponding messages (@ref
 * GNSSInfoMessage, @ref GNSSSatelliteMessage, etc.) may be associated using
 * their @ref p1_time values.
 */
struct PoseMessage {
  /** The time of the message, in P1 time (beginning at power-on). */
  Timestamp p1_time;

  /** The GPS time of the message, if available, referenced to 1980/1/6. */
  Timestamp gps_time;

  /** The type of this position solution. */
  SolutionType solution_type;

  uint8_t reserved[3] = {0};

  /**
   * The WGS-84 geodetic latitude, longitude, and altitude (in degrees/meters).
   */
  double lla_deg[3] = {NAN, NAN, NAN};

  /**
   * The position standard deviation (in meters), resolved with respect to the
   * local ENU tangent plane: east, north, up.
   */
  float position_std_enu_m[3] = {NAN, NAN, NAN};

  /**
   * The platform attitude (in degrees), if known, described as intrinsic
   * Euler-321 angles (yaw, pitch, roll) with respect to the local ENU tangent
   * plane. Set to `NAN` if attitude is not available.
   *
   * @note
   * The platform body axes are defined as +x forward, +y left, and +z up. A
   * positive yaw is a left turn, positive pitch points the nose of the vehicle
   * down, and positive roll is a roll toward the right. Yaw is measured from
   * east in a counter-clockwise direction. For example, north is +90 degrees
   * (i.e., `heading = 90.0 - ypr_deg[0]`).
   */
  double ypr_deg[3] = {NAN, NAN, NAN};

  /**
   * The attitude standard deviation (in degrees): yaw, pitch, roll.
   */
  float ypr_std_deg[3] = {NAN, NAN, NAN};

  /**
   * The platform velocity (in meters/second), resolved in the body frame. Set
   * to `NAN` if attitude is not available for the body frame transformation.
   */
  double velocity_body_mps[3] = {NAN, NAN, NAN};

  /**
   * The velocity standard deviation (in meters/second), resolved in the body
   * frame.
   */
  float velocity_std_body_mps[3] = {NAN, NAN, NAN};

  /** The estimated aggregate 3D protection level (in meters). */
  float aggregate_protection_level_m = NAN;
  /** The estimated 2D horizontal protection level (in meters). */
  float horizontal_protection_level_m = NAN;
  /** The estimated vertical protection level (in meters). */
  float vertical_protection_level_m = NAN;
};

/**
 * @brief Auxiliary platform pose information (@ref MessageType::POSE_AUX).
 * @ingroup messages
 */
struct PoseAuxMessage {
  /** The time of the message, in P1 time (beginning at power-on). */
  Timestamp p1_time;

  /**
   * The position standard deviation (in meters), resolved in the body frame.
   * Set to `NAN` if attitude is not available for the body frame
   * transformation.
   */
  float position_std_body_m[3] = {NAN, NAN, NAN};

  /**
   * The 3x3 position covariance matrix (in m^2), resolved in the local ENU
   * frame. Values are stored in row-major order.
   */
  double position_cov_enu_m2[9] = {NAN};

  /**
   * The platform body orientation with respect to the local ENU frame,
   * represented as a quaternion with the scalar component last (x, y, z, w).
   */
  double attitude_quaternion[4] = {NAN, NAN, NAN, NAN};

  /**
   * The platform velocity (in meters/second), resolved in the local ENU frame.
   */
  double velocity_enu_mps[3] = {NAN, NAN, NAN};

  /**
   * The velocity standard deviation (in meters/second), resolved in the local
   * ENU frame.
   */
  float velocity_std_enu_mps[3] = {NAN, NAN, NAN};
};

/**
 * @brief Information about the GNSS data used in the @ref PoseMessage with the
 *        corresponding timestamp (@ref MessageType::GNSS_INFO).
 * @ingroup messages
 */
struct GNSSInfoMessage {
  static constexpr uint32_t INVALID_REFERENCE_STATION = 0xFFFFFFFF;

  /** The time of the message, in P1 time (beginning at power-on). */
  Timestamp p1_time;

  /** The GPS time of the message, if available, referenced to 1980/1/6. */
  Timestamp gps_time;

  /** The P1 time of the last differential GNSS update. */
  Timestamp last_differential_time;

  /** The ID of the differential base station, if used. */
  uint32_t reference_station_id = INVALID_REFERENCE_STATION;

  /** The geometric dilution of precision (GDOP). */
  float gdop = NAN;
  /** The position dilution of precision (PDOP). */
  float pdop = NAN;
  /** The horizontal dilution of precision (HDOP). */
  float hdop = NAN;
  /** The vertical dilution of precision (VDOP). */
  float vdop = NAN;

  /** GPS time alignment standard deviation (in seconds). */
  float gps_time_std_sec = NAN;
};

/**
 * @brief Information about the individual satellites used in the @ref
 *        PoseMessage and @ref GNSSInfoMessage with the corresponding timestamp
 *        (@ref MessageType::GNSS_SATELLITE).
 * @ingroup messages
 *
 * This message is followed by `N` @ref SatelliteInfo objects, where `N` is
 * equal to @ref num_satellites. For example, a message with two satellites
 * would be serialized as:
 *
 * ```
 * {MessageHeader, GNSSSatelliteMessage, SatelliteInfo, SatelliteInfo, ...}
 * ```
 */
struct GNSSSatelliteMessage {
  /** The time of the message, in P1 time (beginning at power-on). */
  Timestamp p1_time;

  /** The GPS time of the message, if available, referenced to 1980/1/6. */
  Timestamp gps_time;

  /** The number of known satellites. */
  uint16_t num_satellites = 0;

  uint8_t reserved[2] = {0};
};

/**
 * @brief Information about an individual satellite (see @ref
 *        GNSSSatelliteMessage).
 *
 * For satellites where @ref usage is 0, the satellite may either be currently
 * tracked by the receiver but not used for navigation, or may just be expected
 * according to available ephemeris data.
 */
struct SatelliteInfo {
  /**
   * @defgroup satellite_usage Bit definitions for the satellite usage bitmask
   *           (@ref SatelliteInfo::usage).
   * @{
   */
  static constexpr uint8_t SATELLITE_USED = 0x01;
  /** @} */

  /** The GNSS system to which this satellite belongs. */
  SatelliteType system = SatelliteType::UNKNOWN;

  /** The satellite's PRN (or slot number for GLONASS). */
  uint8_t prn = 0;

  /**
   * A bitmask specifying how this satellite was used in the position solution.
   * Set to 0 if the satellite was not used. See @ref satellite_usage.
   */
  uint8_t usage = 0;

  uint8_t reserved = 0;

  /** The azimuth of the satellite (in degrees). */
  float azimuth_deg = NAN;

  /** The elevation of the satellite (in degrees). */
  float elevation_deg = NAN;
};

#pragma pack(pop)

} // namespace messages
} // namespace fusion_engine
} // namespace point_one
