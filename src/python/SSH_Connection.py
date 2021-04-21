import paramiko
import configparser


class SSH_Connection:
    def __init__(self, creds):
        """
        The credentials (creds) argument assumes you have loaded up your config
        file using configparser.

        This can be achieved by doing the following (assuming you have a
        config.ini file in your home directory with all of the ports,
        usernames and passwords):

        Eg.

        path_to_config_file = "/home/bruno/config.ini"
        config = configparser.ConfigParser()
        config.read(path_to_config_file)


        The 'config' variable can now be used as the 'creds' argument when
        invoking this method
        """

        config = configparser.ConfigParser()
        config.read(creds)
        self.creds = config
        self.bbb_connection = None
        self.nuc_connection = None
        self.server_connection = None
        self.server_transport = None
        self.server_channel = None
        self.login_error_message = \
            "Error logging in. Please check that you have the right \
            credentials in your config.ini file"

    def SSH_server(self):
        """
        Initiates server SSH connection

        Returns a paramiko SSH object which can be used to execute shell
        commands using the exec_command() method.

        EG.
        server.exec_command(cmd)

        """

        self.server_connection = paramiko.SSHClient()
        self.server_connection.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self.server_connection.connect(self.creds['Server']['Address'],
                                           username=self.creds['Server']['Username'],
                                           password=self.creds['Server']['Password'])

            print("Connected to server")

        except paramiko.AuthenticationException:
            print("Server_SSH Error: " + self.login_error_message)
            self.server_connection = None
            return

    def SSH_NUC(self):
        """
        Initiates nuc SSH connection

        Returns a paramiko SSH object which can be used to execute
        shell commands using the exec_command() method.

        EG.
        nuc.exec_command(cmd)

        """

        self.nuc_connection = paramiko.SSHClient()
        self.nuc_connection.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self.nuc_connection.connect(self.creds['NUC']['Address'],
                                        port=int(self.creds['NUC']['Port']),
                                        username=self.creds['NUC']['Username'],
                                        password=self.creds['NUC']['Password'])
            print("Connected to NUC")

        except paramiko.AuthenticationException:
            print("NUC_SSH Error: " + self.login_error_message)
            self.nuc_connection = None
            return

    def SSH_BBB(self):
        """
        Initiates bbb SSH connection

        Returns a paramiko SSH object which can be used to execute shell
        commands using the exec_command() method.

        EG.
        bbb.exec_command(cmd)

        """

        self.bbb_connection = paramiko.SSHClient()
        self.bbb_connection.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self.bbb_connection.connect(self.creds['BBB']['Address'],
                                        port=int(self.creds['BBB']['Port']),
                                        username=self.creds['BBB']['Username'],
                                        password=self.creds['BBB']['Password'])
            print("Connected to BBB")
        except paramiko.AuthenticationException:
            print("BBB_SSH Error: " + self.login_error_message)
            self.bbb_connection = None
            return

    def SSH_BBB_tunnel(self):
        """
        Initiates bbb remote SSH connection
        (ie. can be opened from your computer)

        Server connection is needed to run this first

        Returns a paramiko SSH object which can be used to
        execute shell commands using the exec_command() method.

        EG.
        bbb.exec_command(cmd)

        """

        if self.server_connection is None:
            print("BBB_SSH Error: Server connection is needed to ssh into BBB remotely.")
            print("Obtain one by running the SSH_server method first")
            return

        self.server_transport = self.server_connection.get_transport()
        dest_addr = (self.creds['BBB']['Address'], int(self.creds['BBB']['Port']))
        local_addr = (self.creds['Server']['Address'], int(self.creds['Server']['Port']))

        try:
            self.server_channel = self.server_transport.open_channel('direct-tcpip', dest_addr, local_addr)
        except paramiko.ChannelException:
            print("BBB_SSH Error: Could not open remote server channel.")
            print("Please ensure you have the right credentials in your config.ini file")
            return

        self.bbb_connection = paramiko.SSHClient()
        self.bbb_connection.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self.bbb_connection.connect(self.creds['BBB']['Address'],
                                        username=self.creds['BBB']['Username'],
                                        password=self.creds['BBB']['Password'],
                                        sock=self.server_channel)
            print("Connected to BBB through remote computer")
        except paramiko.AuthenticationException:
            print(self.login_error_message)
            self.bbb_connection = None

    def SSH_NUC_tunnel(self):
        """
        Initiates nuc remote SSH connection (ie. can be opened from your computer)

        Server connection is needed to run this first

        Returns a paramiko SSH object which can be used to execute shell commands
        using the exec_command() method.

        EG.
        nuc.exec_command(cmd)

        """
        if self.server_connection is None:
            print("NUC_SSH Error: Server connection is needed to ssh into NUC remotely.")
            print("Obtain one by running the SSH_server method first")
            return

        self.server_transport = self.server_connection.get_transport()
        dest_addr = (self.creds['NUC']['Address'], int(self.creds['NUC']['Port']))
        local_addr = (self.creds['Server']['Address'], int(self.creds['Server']['Port']))

        try:
            self.server_channel = self.server_transport.open_channel('direct-tcpip', dest_addr, local_addr)
        except paramiko.ChannelException:
            print("NUC_SSH Error: Could not open remote server channel.")
            print("Please ensure you have the right credentials in your config.ini file")
            return

        self.nuc_connection = paramiko.SSHClient()
        self.nuc_connection.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self.nuc_connection.connect(self.creds['NUC']['Address'],
                                        username=self.creds['NUC']['Username'],
                                        password=self.creds['NUC']['Password'],
                                        sock=self.server_channel)
            print("Connected to NUC through remote computer")
        except paramiko.AuthenticationException:
            print("NUC_SSH Error: " + self.login_error_message)
            self.nuc_connection = None

    def close_SSH(self):
        """
        Closes the SSH connections to the BBB, NUC, Server

        bbb - SSH connection to the BBB. Obtained by running SSH_BBB
              or SSH_BBB_tunnel if running remotely
        nuc - SSH connection to the NUC. Obtained by running SSH_NUC
              or SSH_NUC_tunnel if running remotely
        server - SSH connection to the server. Obtained by running SSH_server
        """
        if self.bbb_connection is not None:
            self.bbb_connection.close()
            self.bbb_connection = None
            print("BBB Connection closed")
        if self.nuc_connection is not None:
            self.nuc_connection.close()
            self.nuc_connection = None
            print("NUC Connection closed")
        if self.server_connection is not None:
            self.server_connection.close()
            self.server_connection = None
            self.server_transport = None
            self.server_channel = None
            print("Server Connection closed")
        return
