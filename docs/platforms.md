# Platforms and simulation

The final goal of the library is to work on as many platforms as possible. To configure the library for a specific platform, the right features have to be enabled. 

The current platforms and features enabled are
- "rasp": Raspberry Pi and similar controllers

```toml
# platform features
rasp = [ "stepper_lib/rasp" ]
```

If no platform is selected the library automatically goes into simulation mode. In simulation mode, no movements will be executed, but all calculations will be done. Which means that for example GCode scripts can be "debugged" in advance.