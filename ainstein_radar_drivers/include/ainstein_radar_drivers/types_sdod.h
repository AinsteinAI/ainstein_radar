#ifndef types_SDOD_H_
#define COMMON_H_

typedef enum radar_message_type {
  no_type = -1,
  CAN_cartesian_2D_revA_T = 12, 
  CAN_raw_cluster_2D_revA_T = 13,
  CAN_tracked_cluster_2D_revA_T = 14
 } radar_message_type_t;

#endif