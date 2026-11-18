{# The Bambu Lab X2D's filament change, layercraft's own, between two passes of a layer: the
   head up 3 mm, the old filament cut and flushed back into the AMS, the new one loaded and
   flushed, the head back up, the detectors set for the new filament, and its auxiliary fans as
   the layer change sets them, the printer's chamber cooling once past them. Expanded with the
   settings and current_filament_id and next_filament_id (the filaments, from 0),
   current_nozzle_id, next_nozzle_id, current_hotend and next_hotend (the nozzle printed with),
   old_filament_temp and new_filament_temp, flush_length (mm of filament the change flushes),
   long_retraction_when_cut and retraction_distance_when_cut, long_retraction_when_ec and
   retraction_distance_when_ec (how far the old filament is pulled back when cut and when it
   leaves the extruder), max_layer_z, layer_z and layer_num.

   Written from what the maker's change gave on compiles varying one setting at a time, in its
   order. The flush runs at the filament's flush speed in mm³/s, F in mm/min of 1.75 mm
   filament (2.4053 mm²), and takes SYNC T seconds, 5 for every 125 mm. A power loss resumes
   through the M622 J1 branch. Bambu does not document its commands; their roles come from its
   own notes and from its owners. #}
M620 S{next_filament_id}A B H{next_hotend} ; the change begins
G1 Z{max_layer_z + 3} F1200 ; the head up, off the part
M400
M106 P1 S0 ; the part fan off
M620.10 A0 F{flush_volumetric_speeds[current_filament_id] / 2.4053 * 60} L{flush_length} H{nozzle_diameter_at_nozzle_id[current_nozzle_id]} T{flush_temperatures[current_filament_id]} P{old_filament_temp} S1 ; the old filament's flush
M620.10 A1 F{flush_volumetric_speeds[next_filament_id] / 2.4053 * 60} L{flush_length} H{nozzle_diameter_at_nozzle_id[next_nozzle_id]} T{flush_temperatures[next_filament_id]} P{new_filament_temp} S1 ; the new filament's
M620.15 C{new_filament_temp - filament_cooling_before_tower[next_filament_id]} ; the nozzle cooled before the new filament prints
{if long_retraction_when_cut}
M620.11 P1 L0 I{current_filament_id} B{current_hotend} E-{retraction_distance_when_cut} F{max(flush_volumetric_speeds[current_filament_id] / 2.4053 * 60, 200)} ; the old filament pulled back far when cut
{else}
M620.11 P0 L0 I{current_filament_id} B{current_hotend} E0 ; the old filament cut where it is
{endif}
{if long_retraction_when_ec}
M620.11 K1 I{current_filament_id} B{current_hotend} R{retraction_distance_when_ec} F{max(flush_volumetric_speeds[current_filament_id] / 2.4053 * 60, 200)} ; and pulled back as it leaves the extruder
{else}
M620.11 K0 I{current_filament_id} B{current_hotend} R0
{endif}
M620.22 I{next_filament_id} P1 ; a runout purges on its own
T{next_filament_id} H{next_hotend} ; the new filament loaded
SYNC T{ceil(flush_length / 125) * 5} ; and flushed
M1002 set_filament_type:{filament_type[next_filament_id]}
M400
M83
M620.10 R{retract_length_toolchange[filament_map[next_filament_id] - 1]}
M628 S0
M629
M400
M983.3 F{filament_max_volumetric_speed[next_filament_id] / 2.4} A0.4 R{retract_length_toolchange[filament_map[next_filament_id] - 1]} ; its flow calibrated
M400
G1 Z{max_layer_z + 3} F3000
M621 S{next_filament_id}A B ; the change ends
M622.1 S0
M1002 judge_flag powerloss_resume_flag
M622 J1
M983.3 F{filament_max_volumetric_speed[next_filament_id] / 2.4} A0.4 R{retract_length_toolchange[filament_map[next_filament_id] - 1]}
M400
G1 Z{max_layer_z + 3} F3000
M1002 set_flag powerloss_resume_flag=0
M623
M620.6 I{next_filament_id} H{next_hotend} W1 ; the AMS watches for printing in the air
{if filament_type[next_filament_id] == "TPU"}
M1015.3 S1 H{nozzle_diameter_at_nozzle_id[next_nozzle_id]} ; the clog detector, for TPU
{else}
M1015.3 S0 ; no clog detector
{endif}
{if filament_type[next_filament_id] == "PLA" || filament_type[next_filament_id] == "PETG" || filament_type[next_filament_id] == "PLA-CF" || filament_type[next_filament_id] == "PETG-CF"}
M1015.4 S1 K1 H{nozzle_diameter_at_nozzle_id[next_nozzle_id]} ; the extruder watches for printing in the air
{else}
M1015.4 S0 K0 H{nozzle_diameter_at_nozzle_id[next_nozzle_id]} ; no watch for printing in the air
{endif}
{if layer_z <= initial_layer_print_height + 0.001}
M204 S[initial_layer_acceleration] ; the first layer's acceleration
{else}
M204 S[travel_acceleration] ; the travel acceleration
{endif}
G1 Y256 F18000
{# The auxiliary fans as the layer change gives them for the new filament's layer L = layer_num
   + 1, and the printer's chamber cooling at any layer past them. #}
{if layer_num + 1 <= close_additional_fan_first_x_layers[next_filament_id]}
M106 P2 S{first_x_layer_fan_speed[next_filament_id] * 255 / 100} ; the auxiliary part fan
M106 P10 S{first_x_layer_fan_speed[next_filament_id] * 255 / 100} ; the second auxiliary part fan
{elsif layer_num + 1 < additional_fan_full_speed_layer[next_filament_id]}
M106 P2 S{(first_x_layer_fan_speed[next_filament_id] + (additional_cooling_fan_speed[next_filament_id] - first_x_layer_fan_speed[next_filament_id]) * (layer_num + 1 - close_additional_fan_first_x_layers[next_filament_id]) / (additional_fan_full_speed_layer[next_filament_id] - close_additional_fan_first_x_layers[next_filament_id])) * 255 / 100}
M106 P10 S{(first_x_layer_fan_speed[next_filament_id] + (additional_cooling_fan_speed[next_filament_id] - first_x_layer_fan_speed[next_filament_id]) * (layer_num + 1 - close_additional_fan_first_x_layers[next_filament_id]) / (additional_fan_full_speed_layer[next_filament_id] - close_additional_fan_first_x_layers[next_filament_id])) * 255 / 100}
{elsif min_vitrification_temperature <= 50}
M142 P1 R30 S40 U{max_additional_fan / 100} V1.0 O45 ; the chamber cooling, for PLA and TPU
{else}
M142 P1 R50 S55 U{max_additional_fan / 100} V0.5 O55 ; the chamber cooling, for PETG
{endif}
