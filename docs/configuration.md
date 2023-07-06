# Configuration

The library includes methods for parsing JSON configuration files (file extension ".conf.json"). Out of these files, all the constants for a previously defined robot can be parsed. 

Example configuration file:

```json
{
  "name": "SyArm_Mk1",
  "conf_version": "0.0.1/2023/02/21",
  "author": "Samuel Nösslböck",

  "lk": {
    "u": 12,
    "s_f": 1.5
  },

  "anchor": [ 0.0, 0.0, 100.0 ],
  "dims": [
    [ 0.0, 0.0, 15.0 ],
    [ 0.0, 285.0, 0.0 ],
    [ 0.0, 285.0, 0.0 ],
    [ 0.0, 45.0, 0.0 ]
  ],
  "axes": [
    [ 0.0, 0.0, 1.0 ],
    [ 1.0, 0.0, 0.0 ],
    [ 1.0, 0.0, 0.0 ],
    [ 1.0, 0.0, 0.0 ]
  ],

  "comps": [
    {
      "name": "Base",
      "type_name": "syact::comp::gear_bearing::GearJoint",
      "obj": {
        "ctrl": {
          "consts": "MOT_17HE15_1504S",
          "pin_dir": 17,
          "pin_step": 26
        },
        "ratio": 0.08333
      },
      "sim": {
        "mass": 0.2,
        "fric": 2.0
      },
      "meas": {
        "pin": 16,
        "set_val": 0.0,
        "dist": 0.0
      },
      "limit": {
        "vel": 5.0,
        "min": -3.14,
        "max": 3.14
      }
    },
// ... 
```