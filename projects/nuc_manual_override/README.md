## NUC Manual Override
Allows the manual input of values to control the rudder and winch. Nuc Eth Listener must be running at the same time.

Be aware that the rudder angle is in __*radians*__ with the range $[-\frac{\pi}{4}, \frac{\pi}{4}]$ while the winch and
jib position follow this convention:
```
Integer range between position 0 (0th rotation (0 degrees), fully hauled) and position 360 
(8th rotation (8*360 degrees), fully released sail).
```

# How to run
`./nuc_manual_override [-h / --help] [-o / --once] [-t / --test]`
* `-h / --help`: Optional flag. Displays help message.
* `-o / --once`: Optional flag. Indicates that you just want to run the program once and send values once. You need to include the angles you wish to send as an argument in this case. Ex. `./nuc_manual_override -o "12.3 45 67"`
* `-t / --test`: Optional flag. Set if you want to write to a testing rostopic separate from the NUC Eth Listener. If you give it an argument, for example. `./nuc_manual_override -t/test_rostopic`, it will publish to `/test_rostopic` (note, you cannot have a space after -t). Without an argument, it publishes to `/nuc_manual_override_test` by default.

All of the above flags are optional. The `-o` and `-t` flags can be used simultaneously.

# Testing
* Run Nuc Ethernet Listener: `./nuc_eth_listener <IP> <PORT>`, with whichever IP and port you want. It does not matter for this test. For example: `./nuc_eth_listener 127.0.0.1 5555`.
* Make sure `roscore` is running.
* Source the ROS message files: `source build/devel/setup.bash` or `source build/devel/setup.zsh`.
* Run the following command: `rostopic pub /rudder_winch_actuation_angle sailbot_msg/actuation_angle 12.3 45 67 -r 1` This simulates messages being sent via the `rudder_winch_actuation_angle` at a rate of 1 Hz. 
* Run the manual override: `./nuc_manual_override`. 

As soon as you enter a value into the manual override, for example with `"0.39 90 52"` the output from the `nuc_eth_listener` should stop accepting `12.3 45 67` values from the `/rudder_winch_actuation_angle` topic. Entering `stop` or `exit` should resume the 1 Hz messages. After entering `stop`, entering any correct set of angles will have the listener prioritize the manual override and ignore the 1 Hz messages again.

For more concrete results, in another window, run:
* `./network_table_server &`
* `./bbb_eth_listener <IP> <PORT>`, where IP and PORT are the same what you passed to the `nuc_eth_listener`.

The output of `bbb_eth_listener` should match that of `nuc_eth_listener`.