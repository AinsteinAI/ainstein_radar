#ifndef types_o79_H_
#define COMMON_H_

#define CAN_NUM_TRACK_CART_REV_C_PER_FRAME  10

typedef enum radar_message_type {
  no_type = -1,
  raw_spherical = 0,
  tracked_spherical = 1, // not supported for O-79
  bounding_box = 2,
  tracked_cartesian = 4,
  ground_cartesian = 5,
  raw_sphere_16bit_pwr = 6,
  alarm_status = 7,
  filtered_point_cloud = 8,
  tracked_cartesian_low_res = 9,
  track_cart_rev_c = 10
} radar_message_type_t;

#endif