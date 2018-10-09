/*
 Copyright <2018> <Ainstein, Inc.>

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 conditions and the following disclaimer in the documentation and/or other materials provided
 with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CONFIG_T79_BSD_H_
#define CONFIG_T79_BSD_H_

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

const int RADAR_SEND_SPEED = 0x130;
const int RADAR_SPEED_EFFECTIVE_POS = 0x01;

const int RADAR_SET_DISABLE_BSD = 0x00;
const int RADAR_SET_ENABLE_BSD = 0x01;

// Message IDs specific to each type of BSD firmware radar:
const std::map<RadarType, int> stop_ret = {
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

const std::map<RadarType, int> start_frame = {
                                               {
                                                 KANZA,
                                                 0x421 },
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

}

#endif // CONFIG_T79_BSD_H_
