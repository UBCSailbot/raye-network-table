# Supervisor Configuration

Configuration files for supervisord, which we are using to run our programs on startup as well as providing a means to automatically recover and restart a program if it crashes.

All `.conf` files should be symlinked under `/etc/supervisor/conf.d/` with the exception of `land.conf`. This is because the password will need to be manually inserted into the land satellite listener command.

Install with `sudo apt install supervisord`.
