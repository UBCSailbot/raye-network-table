# Systemd Configurations

Configuration files for Systemd, which we are using to run our programs on startup as well as providing a means to automatically recover and restart a program if it crashes.

All `.service` files should be symlinked under `/etc/systemd/system/` with the exception of `ls_listener.service`.
This is because the password will need to be manually inserted into the land satellite listener command.

After symlinking, make sure to reload with `systemctl daemon-reload` and enable the services with `systemctl enable <service name>`.
Additionally, services can be manuallyed started or stopped with `systemctl [start/stop] <service name>`.
To view the outputs of a service, run `journalctl -u <service name>`.

If any commands fail, run them with `sudo`.
