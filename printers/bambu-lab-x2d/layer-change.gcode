{# The Bambu Lab X2D's layer change, layercraft's own, before each layer's first pass: the two
   auxiliary part fans through the first layers, then the printer's own chamber cooling, and the
   layer counter. Expanded with the settings, layer_num (the layer, from 0), current_filament_id
   (the filament loaded) and, over the print's filaments, min_vitrification_temperature (the
   lowest softening point) and max_additional_fan (the fastest auxiliary fan).

   Layer L = layer_num + 1 of a filament that keeps its auxiliary fans at s0 % for its first n
   layers and reaches s1 % at layer f: the fans run at s0 through layer n, then climb in a
   straight line, s0 + (s1 - s0)(L - n)/(f - n), a fan's 100 % being S255. At layer
   max(n + 1, f) the printer takes them over (M142), never under the fastest s1, faster as the
   chamber warms, to keep the chamber below what the filaments stand: a band for those softening
   at 50 °C or less (PLA, TPU), another above (PETG), measured with the 0.4 mm nozzle. The
   letters read as the chamber temperatures where the cooling starts (R) and peaks (S), the
   fans' range (U to V) and a limit (O); Bambu does not document them. #}
{if layer_num + 1 < max(close_additional_fan_first_x_layers[current_filament_id] + 1, additional_fan_full_speed_layer[current_filament_id])}
M106 P2 S{(layer_num + 1 <= close_additional_fan_first_x_layers[current_filament_id] ? first_x_layer_fan_speed[current_filament_id] : first_x_layer_fan_speed[current_filament_id] + (additional_cooling_fan_speed[current_filament_id] - first_x_layer_fan_speed[current_filament_id]) * (layer_num + 1 - close_additional_fan_first_x_layers[current_filament_id]) / (additional_fan_full_speed_layer[current_filament_id] - close_additional_fan_first_x_layers[current_filament_id])) * 255 / 100} ; the auxiliary part fan
M106 P10 S{(layer_num + 1 <= close_additional_fan_first_x_layers[current_filament_id] ? first_x_layer_fan_speed[current_filament_id] : first_x_layer_fan_speed[current_filament_id] + (additional_cooling_fan_speed[current_filament_id] - first_x_layer_fan_speed[current_filament_id]) * (layer_num + 1 - close_additional_fan_first_x_layers[current_filament_id]) / (additional_fan_full_speed_layer[current_filament_id] - close_additional_fan_first_x_layers[current_filament_id])) * 255 / 100} ; the second auxiliary part fan
{elsif layer_num + 1 == max(close_additional_fan_first_x_layers[current_filament_id] + 1, additional_fan_full_speed_layer[current_filament_id])}
{if min_vitrification_temperature <= 50}
M142 P1 R30 S40 U{max_additional_fan / 100} V1.0 O45 ; the chamber cooling, for PLA and TPU
{else}
M142 P1 R50 S55 U{max_additional_fan / 100} V0.5 O55 ; the chamber cooling, for PETG
{endif}
{endif}
M73 L{layer_num + 1} ; the layer counter, from 1
M991 S0 P{layer_num} ; the layer's change, from 0
