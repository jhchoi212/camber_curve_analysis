# SLA Suspension design tool. Jonathan Choi 3/11/24
## This repo is designed to aid in the design of a front and rear SLA suspension setup
Files contained in this repo will consume user defined parameters and then calculate camber curves, instant roll centers, roll center, and other usefull measures.

In order to utilize this tool kit, the user must populate the following design parameters in the `sla_params_config.YAML` file. 
Note that only the ones mentioned below are necessary and parameters in the Populating data section will auto-generate when the tools have been run
The main parameters for the suspension design can be altered in the `sla_params_config.YAML`
Parameters include
* `subframe_length` [in]
* Uprights
  * `spindle_length` [in]
  * `hub_center_from_lca` [in]
* `lca` (Lower Control Arm)
  * `lca_length` [in]
  * `lca_mounts` (x, y). Origin
  * `lca_sweep` (lower:step:upper). [deg]
* `uca` (Upper Control Arm)
  * `uca_length` [in]
  * `uca_mounts` (x, y). 
  * `uca_sweep` (lower:step:upper). [deg]
* `ride_height`
  * `lca_phi` [deg]
  * `uca_phi` [deg] -DO NOT FILL THIS OUT-
* `wheel`
  * `radius` [in]
  * `width` [in]
  * `backspace` [in]
* `tire`
  * `width` [in]
  * `ratio` [~]

## Populating data in the correct order
The tools in this repo are meant to be used sequenctially, in order to follow the full developmental phase of designing SLA suspension
The first tool that the user needs to run after populating the `sla_params_config.YAML` file is the `camber_curves_sla.m` file. 
This file will calculate the matched points on the two sweeping arcs of the LCA and UCA to plot and save camber curve data.
The following fields in the `sla_params_config.YAML` file will be updated
* `camber_data` (lca_x, lca_y, uca_x, uca_y) [in]
* `ride_height`
  * `uca_phi` [deg] 

After the `camber_curves_sla.m` file has been run, the user can then move onto the `roll_center_analysis.m` file in order to find the roll center at ride height.
The following fields will be populated in the `sla_params_config.YAML` file:
* `roll_center`
  * `front` (x, y) [in]
  * `rear` (x, y) [in]
