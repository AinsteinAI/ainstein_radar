#ifndef types_o79_H_
#define COMMON_H_

typedef enum target_type {
  no_type = -1,
  raw_spherical = 0,
  tracked_spherical = 1,
  bounding_box = 2,
  tracked_cartesian = 4,
  ground_cartesian = 5,
  raw_sphere_16bit_pwr = 6  
} target_type_t;

#endif