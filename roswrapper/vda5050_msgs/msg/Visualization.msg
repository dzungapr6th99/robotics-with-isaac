# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# This repository implements data types and logic specified in the VDA5050
# protocol, which is specified here
# https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md

# AGV position and/or velocity for visualization purposes. Fields are optional
# for bandwidth reasons; populate the values you have available.

# headerId of the message. The headerId is defined per topic and incremented
# by 1 with each sent (but not necessarily received) message.
int32 header_id

# Timestamp in ISO8601 format (YYYY-MM-DDTHH:mm:ss.ffZ).
string timestamp

# Version of the protocol [Major].[Minor].[Patch] (e.g. 1.3.2)
string version

# Manufacturer of the AGV
string manufacturer

# Serial number of the AGV.
string serial_number

# The AGV position.
vda5050_msgs/AGVPosition agv_position

# The AGV velocity in vehicle coordinates.
vda5050_msgs/Velocity velocity
