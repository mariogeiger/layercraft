{# The Bambu Lab X2D's start, layercraft's own: a tune, the machine reset and the logo lamp on,
   the chamber cooled, the head homed, the plate checked, the first filament loaded from the AMS,
   the flow calibrated, the nozzle wiped, Z found, the plate levelled where the first layer
   lies, the vibrations measured, the nozzles' offset calibrated, and a purge line. Expanded
   with the settings and initial_filament_id (the first filament), initial_nozzle_id and
   initial_no_support_hotend (the nozzle it prints with), bed_temperature_initial_layer_single
   (the plate's first-layer temperature), first_layer_print_min and first_layer_print_size (the
   first layer's extent), and max_print_z (the part's top).

   Written from what the maker's start gave on compiles varying one setting at a time, in its
   order. A judge_flag sets the M622 J branch the printer runs from what the print asks: flow
   calibration (extrude_cali_flag), levelling (g29_before_print_flag), the nozzles' offset
   (auto_cali_toolhead_offset_flag). layercraft never heats the chamber and prints with one
   nozzle, so the second nozzle's offset is calibrated at filament 0's temperature. Bambu does
   not document its commands; their roles come from its own notes and from its owners. #}
M140 S{bed_temperature_initial_layer_single} ; the plate heats first
M993 A0 B0 C0 ; the nozzle camera's checks off
M400
{# The start tune, kept by the owner's choice. #}
M17
M400 S1
M1006 S1
M1006 A53 B9 L50 C53 D9 M50 E53 F9 N50
M1006 A56 B9 L50 C56 D9 M50 E56 F9 N50
M1006 A61 B9 L50 C61 D9 M50 E61 F9 N50
M1006 A53 B9 L50 C53 D9 M50 E53 F9 N50
M1006 A56 B9 L50 C56 D9 M50 E56 F9 N50
M1006 A61 B18 L50 C61 D18 M50 E61 F18 N50
M1006 W
M1012.1 T1100
M620 M ; the AMS's slot mapping on
M622.1 S0
G383.4
G91 ; Z off its end stop
G380 S2 Z22 F1200
G380 S2 Z-12 F1200
G90
M204 S10000 ; the machine reset
M630 S0 P1
G90
M17 D ; the motors' default currents
M960 S5 P1 ; the logo lamp on, by the owner's choice
M220 S100 ; speed at 100 %
M1002 set_gcode_claim_speed_level: 5
M221 S100 ; flow at 100 %
M73.2 R1.0 ; remaining time unscaled
G29.1 Z0 ; no Z trim
M983.1 M1
M982.2 S1 ; the motors' noise reduction on
M145 P0 ; the air duct in cooling mode
M106 P2 S255 ; the auxiliary part fans and the chamber fan cool the chamber
M106 P10 S255
M106 P3 S127
M1002 gcode_claim_action : 29
M191 S0 ; wait for the chamber
M106 P2 S102
M106 P10 S102
M142 P6 R30 S40 U0.6 V0.8 ; then the printer cools the chamber itself
M1002 gcode_claim_action : 2
M1002 set_filament_type:{filament_type[initial_filament_id]}
G29.2 S0 ; the bed mesh off
M1002 gcode_claim_action : 13 ; X homed, the nozzle wiped
G28 X T300 R
G150.1 F8000
G150.3
M972 S24 P0
M1002 gcode_claim_action : 74 ; foreign objects on the plate
M972 S26 P0 C0
G90
M83
G1 Y128 F30000 ; Z homed at the plate's centre
G1 X128
G28 Z P0 T400
M400
M1002 gcode_claim_action : 11 ; the plate checked, the nozzles cold
M104 S0 T0
M104 S0 T1
M562 P1 E0 B1
M562 P2 E0 B1
M18 E
M400 P200
M1028 S1
M972 S19 P0 ; the plate there
M972 S31 P0 ; the toolhead camera clean
M1002 gcode_claim_action : 73 ; the plate aligned
M972 S34 P0 ; the plate flat
M1028 S0
M562 P1 E1 B1
M562 P2 E1 B1
M17 D
M104 S{nozzle_temperature_initial_layer[initial_filament_id]} T{filament_map[initial_filament_id] % 2} ; the nozzle heats ahead
{if max_print_z >= 145}
G151 P{filament_map[initial_filament_id] % 2} M ; a tall part: nothing under the plate
M1002 gcode_claim_action : 75
G3811 Z{max_print_z}
{endif}
M104 S{nozzle_temperature_initial_layer[initial_filament_id] - 40} A ; the first filament loaded
M400
M211 X0 Y0 Z0 ; the soft limits off
M975 S1 ; input shaping on
G29.2 S0
G150.3
M620.10 A0 F{flush_volumetric_speeds[initial_filament_id] / 2.4053 * 60} H{nozzle_diameter_at_nozzle_id[initial_nozzle_id]} T{flush_temperatures[initial_filament_id]} P{nozzle_temperature_initial_layer[initial_filament_id]} S1
M620.10 A1 F{flush_volumetric_speeds[initial_filament_id] / 2.4053 * 60} H{nozzle_diameter_at_nozzle_id[initial_nozzle_id]} T{flush_temperatures[initial_filament_id]} P{nozzle_temperature_initial_layer[initial_filament_id]} S1
M620.11 P0 L0 I{initial_filament_id} B{initial_no_support_hotend} E0
M620.11 K0 I{initial_filament_id} B{initial_no_support_hotend} R0
M620 S{initial_filament_id}A H{initial_no_support_hotend} B
M620.22 I{initial_filament_id} P1 ; a runout purges on its own
M1002 gcode_claim_action : 4
M1002 set_filament_type:UNKNOWN
M400
T{initial_filament_id} H{initial_no_support_hotend}
M400
M628 S0
M629
M400
M1002 set_filament_type:{filament_type[initial_filament_id]}
M621 S{initial_filament_id}A B
M104 S{nozzle_temperature_initial_layer[initial_filament_id]}
M400
M106 P1 S0
M400
G29.2 S1
M975 S1 ; the flow calibrated, as the print asks
M1002 judge_flag extrude_cali_flag
M622 J0
M983.3 F{filament_max_volumetric_speed[initial_filament_id] / 2.4} A0.4
M623
M622 J1
M1002 set_filament_type:{filament_type[initial_filament_id]}
M1002 gcode_claim_action : 8
M109 S{nozzle_temperature[initial_filament_id]}
G90
M83
M983.3 F{filament_max_volumetric_speed[initial_filament_id] / 2.4} A0.4
M400
M106 P1 S255
M400 S5
M106 P1 S0
G150.3
M623
M622 J2
M1002 set_filament_type:{filament_type[initial_filament_id]}
M1002 gcode_claim_action : 8
M109 S{nozzle_temperature[initial_filament_id]}
G90
M83
M983.3 F{filament_max_volumetric_speed[initial_filament_id] / 2.4} A0.4
M400
M106 P1 S255
M400 S5
M106 P1 S0
G150.3
M623
{if filament_type[initial_filament_id] != "TPU" && filament_type[initial_filament_id] != "PVA"}
M83 ; pulled back, but for the soft and the soluble
G1 E-3 F1800
M400 P500
{endif}
G150.2 ; the nozzle wiped
G150.1 F8000
G150.2
G150.1 F8000
G91
G1 Y-16 F12000 ; away from the waste chute
G90
M400
M104 S{nozzle_temperature_initial_layer[initial_filament_id] - 80} A
M1002 gcode_claim_action : 14
G150 T{nozzle_temperature_initial_layer[initial_filament_id]}
M400
M109 S{filament_type[initial_filament_id] == "PC" ? 170 : 140} A ; cool enough to touch the plate
M106 S0
G91
G1 Z5 F1200
G90
M400
G150.1
M190 S{bed_temperature_initial_layer_single} ; Z found on the hot plate
G383 O0 M1 T140
M400
G90
M83
G0 Y200 F18000
M1002 gcode_claim_action : 54 ; the plate levelled where the first layer lies, as the print asks
M190 S{bed_temperature_initial_layer_single}
M109 S140 A
M106 S0
M1002 judge_flag g29_before_print_flag
M622 J1
M1002 gcode_claim_action : 1
G29 A1 X{first_layer_print_min[0]} Y{first_layer_print_min[1]} I{first_layer_print_size[0]} J{first_layer_print_size[1]} R
M400
M623
M622 J2
M1002 gcode_claim_action : 1
G29 A2 X{first_layer_print_min[0]} Y{first_layer_print_min[1]} I{first_layer_print_size[0]} J{first_layer_print_size[1]} R
M400
M623
M622 J0
G28 R
M623
G29.2 S1
M104 S{nozzle_temperature_initial_layer[initial_filament_id]} A
M1002 gcode_claim_action : 3 ; the vibrations measured at the plate's centre
G90
G1 X128 Y128 F20000
G1 Z5 F1200
M400 P200
M970.3 Q1 A5 K0 O1
M974 Q1 S2 P0
M970.3 Q0 A7 K0 O1
M970.2 Q0 W73 K1 Z0.01
M974 Q0 S2 P0
M975 S1
M400
M104 S{nozzle_temperature_initial_layer[initial_filament_id]} A
G150.3
M1002 judge_flag auto_cali_toolhead_offset_flag ; the nozzles' offset, as the print asks
M622 J0
M1012.5 N1 R1
M623
M622 J1
M1002 gcode_claim_action : 39
M141 S0
M620.17 T0 S{nozzle_temperature_initial_layer[initial_filament_id]} L{initial_filament_id}
M620.17 T1 S{nozzle_temperature_initial_layer[0]} L0
M620 D{initial_no_support_hotend}
G383 O1 T{nozzle_temperature_initial_layer[initial_filament_id]} L{initial_filament_id}
M141 S0
M623
M622 J2
M1002 gcode_claim_action : 39
M141 S0
M620.17 T0 S{nozzle_temperature_initial_layer[initial_filament_id]} L{initial_filament_id}
M620.17 T1 S{nozzle_temperature_initial_layer[0]} L0
M620 D{initial_no_support_hotend}
G383.3 T{nozzle_temperature_initial_layer[initial_filament_id]} L{initial_filament_id}
M141 S0
M623
M104 S{nozzle_temperature_initial_layer[initial_filament_id]} A
G150.3 ; over the waste chute, waiting for the temperatures
M140 S{bed_temperature_initial_layer_single}
M190 S{bed_temperature_initial_layer_single}
M960 S1 P0 ; the lasers off
M960 S2 P0
M106 S0
M1002 gcode_claim_action : 255
M400
M975 S1
M983.4 S0
M211 X0 Y0 Z0 ; the first filament once more
G91
G1 Z6 F1200
G90
M1002 set_filament_type:{filament_type[initial_filament_id]}
M620 S{initial_filament_id}A H{initial_no_support_hotend} B
M620.22 I{initial_filament_id} P1
M400
T{initial_filament_id} H{initial_no_support_hotend}
M400
M628 S0
M629
M400
M621 S{initial_filament_id}A B
{if curr_bed_type == "Textured PEI Plate"}
G29.1 Z{bed_temperature_initial_layer_single > 70 ? -0.003 : 0.002} ; the textured plate's trim
{else}
G29.1 Z{bed_temperature_initial_layer_single > 70 ? 0.017 : 0.022} ; a smooth plate's trim
{endif}
M1002 gcode_claim_action : 51 ; the purge line
G29.2 S1
G90
M83
M400 P50
M500 D1
M400 S3
M109 S{nozzle_temperature_initial_layer[initial_filament_id]}
G0 X100 Y0 F24000
M400
G130 O0 X100 Y-0.2 Z0.6 F{filament_max_volumetric_speed[initial_filament_id] / 2 / 2.4053} L40 E12 D4
G90
G90
M83
G1 Z1
M400
M1002 gcode_claim_action : 0
G29.99
M620.6 I{initial_filament_id} H{initial_no_support_hotend} W1 ; the AMS watches for printing in the air
{if filament_type[initial_filament_id] == "TPU"}
M1015.3 S1 H{nozzle_diameter_at_nozzle_id[initial_nozzle_id]} ; the clog detector, for TPU
{else}
M1015.3 S0 ; no clog detector
{endif}
{if filament_type[initial_filament_id] == "PLA" || filament_type[initial_filament_id] == "PETG" || filament_type[initial_filament_id] == "PLA-CF" || filament_type[initial_filament_id] == "PETG-CF"}
M1015.4 S1 K1 H{nozzle_diameter_at_nozzle_id[initial_nozzle_id]} ; the extruder watches for printing in the air
{else}
M1015.4 S0 K0 H{nozzle_diameter_at_nozzle_id[initial_nozzle_id]} ; no watch for printing in the air
{endif}
