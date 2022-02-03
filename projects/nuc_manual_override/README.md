## NUC Manual Override
Allows the manual input of values to control the rudder and winch. Nuc Eth Listener must be running at the same time.

# How to run
`./nuc_manual_override [-h / --help] [-o / --once] [-t / --test]`
* `-h / --help`: Optional flag. Displays help message.
* `-o / --once`: Optional flag. Indicates that you just want to run the program once and send values once. You need to include the angles you wish to send as an argument in this case. Ex. `./nuc_manual_override -o "12.3 45.6"`
* `-t / --test`: Optional flag. Set if you want to write to a testing rostopic separate from the NUC Eth Listener. If you give it an argument, for example. `./nuc_manual_override -t/test_rostopic`, it will publish to `/test_rostopic` (note, you cannot have a space after -t). Without an argument, it publishes to `/nuc_manual_override_test` by default.

All of the above flags are optional. The `-o` and `-t` flags can be used simultaneously.
