# Log CAN
Simple bash script that executes the `candump can0` command on the BBB, prepends it with a timestamp,
and saves it to a `.log` file under `network-table/scripts/can_logging/logfiles`.

Functionality can be tested with `./log_can.sh -t "python3 test_printer.py"`.
