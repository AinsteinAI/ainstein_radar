#ifndef CONFIG_T79_BSD_H_
#define CONFIG_T79_BSD_H_

namespace ainstein_radar_drivers
{
  
namespace ConfigT79BSD
{

enum RadarType
{
    KANZA = 0,
    TIPI_79_FL,
    TIPI_79_FR,
    TIPI_79_RL,
    TIPI_79_RR,
    N_RADARS
};

const std::map<RadarType, std::string> radar_names = {
                                                       {
                                                         KANZA,
                                                         "KANZA" },
                                                       {
                                                         TIPI_79_FL,
                                                         "TIPI_79_FL" },
                                                       {
                                                         TIPI_79_FR,
                                                         "TIPI_79_FR" },
                                                       {
                                                         TIPI_79_RL,
                                                         "TIPI_79_RL" },
                                                       {
                                                         TIPI_79_RR,
                                                         "TIPI_79_RR" } };

// Message IDs common to all BSD firmware radars:
const int RADAR_START_STOP = 0x100;
const int RADAR_START = 0x01;
const int RADAR_STOP = 0x02;
const int RADAR_CYCLES = 0x00;
const int RESERVED = 0xff; 

const int RADAR_SEND_SPEED = 0x130;
const int RADAR_SPEED_EFFECTIVE_POS = 0x01;

const int RADAR_SET_DISABLE_BSD = 0x00;
const int RADAR_SET_ENABLE_BSD = 0x01;

// Message IDs specific to each type of BSD firmware radar:
 const std::map<RadarType, int> heartbeat_1 = {
                                            {
                                              KANZA,
                                              0xDEAD },
                                            {
                                              TIPI_79_FL,
                                              0x502 },
                                            {
                                              TIPI_79_FR,
                                              0x503 },
                                            {
                                              TIPI_79_RL,
                                              0x504 },
                                            {
                                              TIPI_79_RR,
                                              0x505 } };

 const std::map<RadarType, int> heartbeat_2 = {
                                            {
                                              KANZA,
                                              0xDEAD },
                                            {
                                              TIPI_79_FL,
                                              0x503 },
                                            {
                                              TIPI_79_FR,
                                              0x504 },
                                            {
                                              TIPI_79_RL,
                                              0x505 },
                                            {
                                              TIPI_79_RR,
                                              0x506 } };

 const std::map<RadarType, int> start_frame = {
                                               {
                                                 KANZA,
                                                 0x420 },
                                               {
                                                 TIPI_79_FL,
                                                 0x421 },
                                               {
                                                 TIPI_79_FR,
                                                 0x422 },
                                               {
                                                 TIPI_79_RL,
                                                 0x423 },
                                               {
                                                 TIPI_79_RR,
                                                 0x424 } };

const std::map<RadarType, int> stop_frame = {
                                              {
                                                KANZA,
                                                0x480 },
                                              {
                                                TIPI_79_FL,
                                                0x481 },
                                              {
                                                TIPI_79_FR,
                                                0x482 },
                                              {
                                                TIPI_79_RL,
                                                0x483 },
                                              {
                                                TIPI_79_RR,
                                                0x484 } };


 const std::map<RadarType, int> start_stop_ret = {
                                            {
                                              KANZA,
                                              0x101 },
                                            {
                                              TIPI_79_FL,
                                              0x111 },
                                            {
                                              TIPI_79_FR,
                                              0x121 },
                                            {
                                              TIPI_79_RL,
                                              0x131 },
                                            {
                                              TIPI_79_RR,
                                              0x141 } };

const std::map<RadarType, int> radar_id = {
                                            {
                                              KANZA,
                                              0x101 },
                                            {
                                              TIPI_79_FL,
                                              0x111 },
                                            {
                                              TIPI_79_FR,
                                              0x121 },
                                            {
                                              TIPI_79_RL,
                                              0x131 },
                                            {
                                              TIPI_79_RR,
                                              0x141 } };

const std::map<RadarType, int> tracked_id = {
                                              {
                                                KANZA,
                                                0x490 },
                                              {
                                                TIPI_79_FL,
                                                0x491 },
                                              {
                                                TIPI_79_FR,
                                                0x492 },
                                              {
                                                TIPI_79_RL,
                                                0x493 },
                                              {
                                                TIPI_79_RR,
                                                0x494 } };

const std::map<RadarType, int> raw_id = {
                                          {
                                            KANZA,
                                            0x4A0 },
                                          {
                                            TIPI_79_FL,
                                            0x4A1 },
                                          {
                                            TIPI_79_FR,
                                            0x4A2 },
                                          {
                                            TIPI_79_RL,
                                            0x4A3 },
                                          {
                                            TIPI_79_RR,
                                            0x4A4 } };

const std::map<RadarType, int> bsd_id = {
                                          {
                                            KANZA,
                                            0xDEAD },
                                          {
                                            TIPI_79_FL,
                                            0xDEAD },
                                          {
                                            TIPI_79_FR,
                                            0xDEAD },
                                          {
                                            TIPI_79_RL,
                                            0x453 },
                                          {
                                            TIPI_79_RR,
                                            0x454 } };

} // namespace ConfigT79BSD

} // namespace ainstein_radar_drivers

#endif // CONFIG_T79_BSD_H_
