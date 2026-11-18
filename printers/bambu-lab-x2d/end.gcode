; The Bambu Lab X2D's end, layercraft's own: the nozzle off the part, the heaters and fans off,
; each nozzle's filament back in the AMS, the head parked, the plate lowered gently so the part
; comes off, the air purified when the printer is set to, a short tune, the motors off.
; Expanded with the settings and max_layer_z, the top of the highest layer.
M400 ; every move done
G92 E0
M211 Z1 ; Z's soft limit back on
G90
G1 Z{max_layer_z + 0.4} F900 ; off the last layer
G1 Z{max_layer_z + 10} F900 ; well clear of the part

; Heaters and fans off.
M140 S0 ; the bed
M141 S0 ; the chamber
M106 S0 ; the part fan
M106 P2 S0 ; the auxiliary part fan
M106 P3 S0 ; the chamber fan
M106 P10 S0 ; the second auxiliary part fan

; Each nozzle's filament back into the AMS: the left one's (tool 65279), then the right one's
; (tool 65535); then the head to its parking place and both nozzles off.
M620 S65279 B
T65279
G150.1 F8000
M621 S65279 B
M620 S65535 B
T65535
G150.1 F8000
M621 S65535 B
G150.3
M104 S0 T0
M104 S0 T1

; The plate down to 80 mm, or 10 mm under the top of a taller part, with Z at 40 % of its
; current: a plate that meets something below stalls instead of pushing it.
M400
M17 S ; keep the motors' currents
M17 Z0.4
G1 Z{min(max(max_layer_z + 10, 80), printable_height)} F600
M400 P100
M17 R ; the motors' currents back

; Back to normal what a print may change: the speed, acceleration and time scales, the speed
; level, and the clog and air-printing detectors the start turns on.
M220 S100
M201.2 K1.0
M73.2 R1.0
M1002 set_gcode_claim_speed_level : 0
M1015.3 S0
M1015.4 S0 K0

; The air purified for three minutes when the printer is set to: through its filter (mode 1),
; or out of the chamber (mode 2).
M622.1 S0
M1002 judge_flag print_finish_air_filt_flag
M622 J1
M1002 gcode_claim_action : 66
M145 P1
M106 P10 S255
M400 S180
M106 P10 S0
M623
M622 J2
M1002 gcode_claim_action : 66
M145 P0
M106 P3 S255
M400 S180
M106 P3 S0
M623

; A rising C major arpeggio, C3 E3 G3 C4, on three voices, each a note (MIDI), a length and a
; volume.
M17
M400 S1
M1006 S1
M1006 A48 B10 L50 C48 D10 M50 E48 F10 N50
M1006 A52 B10 L50 C52 D10 M50 E52 F10 N50
M1006 A55 B10 L50 C55 D10 M50 E55 F10 N50
M1006 A60 B30 L50 C60 D30 M50 E60 F30 N50
M1006 W
M400
M18 ; the motors off
M73 P100 R0 ; done
