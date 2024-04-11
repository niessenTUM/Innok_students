
import paramiko
import threading

# clean all python processes on the Raspberry Pi
ssh_cleaner = paramiko.SSHClient()
ssh_cleaner.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh_cleaner.connect('10.21.20.15', port=22, username='ubuntu', password='heros_ubuntu')
stdin, stdout, stderr = ssh_cleaner.exec_command(
    # Kill all python processes
    'sudo pkill -f python3 -9\n sleep 0.1\n'
    # password for sudo#
    'heros_ubuntu\n sleep 0.1\n'
)
ssh_cleaner.close()

class StartSchutzlaserThreadPi(threading.Thread):
    def __init__(self, iD, name):           # Initialize the first thread
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name

        self.sshSchutzlaser = paramiko.SSHClient()
        self.sshSchutzlaser.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    def run(self):
        self.sshSchutzlaser.connect('10.21.20.15', port=22, username='ubuntu', password='heros_ubuntu') 
        # allow ssh to use display by running an X Server, access directory, and run file
        stdin, stdout, stderr = self.sshSchutzlaser.exec_command(
            # Setup X Server to show pygame's output
            'export XAUTHORITY=~/.Xauthority\n sleep 0.1\n export DISPLAY=:0.0\n sleep 0.1\n sudo xhost +\n sleep 0.1\n'
            # Setup ROS environment
            'source /opt/ros/noetic/setup.bash\n sleep 0.1\n export ROS_MASTER_URI=http://10.21.20.10:11311\n sleep 0.1\n export ROS_IP=10.21.20.15\n sleep 0.1\n'
            # # Make sure we have proper environment for running the python code
            # 'echo ROS distro: $ROS_DISTRO\n sleep 0.1\n echo $Python environemnt: $PYTHONPATH\n sleep 0.1\n'
            # Run the file
            'cd /home/ubuntu/Desktop/Communication \n python3 av_combined.py \n')
        #  Save the output that is printed to the console by the program send_data_over_ssh to the variable linearspeed
        for line in iter(stdout.readline, ""):      
            print("SCHUTZLASER: " + stdout.readline())

class StartTrajektoriePi(threading.Thread):
    def __init__(self, iD, name):           # Initialize the first thread
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name

        self.sshTrajektorie = paramiko.SSHClient()
        self.sshTrajektorie.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    def run(self):
        self.sshTrajektorie.connect('10.21.20.15', port=22, username='ubuntu', password='heros_ubuntu')    # Start first SSH connection to innok and enter username and password automatically
        # allow ssh to use display by running an X Server, access directory, and run file
        stdin, stdout, stderr = self.sshTrajektorie.exec_command(
            # Setup X Server to show pygame's output
            'export XAUTHORITY=~/.Xauthority\n sleep 0.1\n export DISPLAY=:0.0\n sleep 0.1\n sudo xhost +\n sleep 0.1\n'
            # Setup ROS environment
            'source /opt/ros/noetic/setup.bash\n sleep 0.1\n export ROS_MASTER_URI=http://10.21.20.10:11311\n sleep 0.1\n export ROS_IP=10.21.20.15\n sleep 0.1\n'
            # # Make sure we have proper environment for running the python code
            # 'echo ROS distro: $ROS_DISTRO\n sleep 0.1\n echo $Python environemnt: $PYTHONPATH\n sleep 0.1\n'
            # Run the file
            'cd /home/ubuntu/Desktop/Communication \n python3 dap_ros_pi_filter.py \n')
        #  Save the output that is printed to the console by the program send_data_over_ssh to the variable linearspeed
        for line in iter(stdout.readline, ""):      
            print("TRAJEKTORIE" + stdout.readline())
                

# Connection to Raspberry
# c_schutzlaser = StartSchutzlaserThread(1, "schutzlaser")
c_schutzlaser_pi = StartSchutzlaserThreadPi(1, "schutzlaser_pi")
c_trajektorie_pi = StartTrajektoriePi(2, "trajektorie_pi")

# Initialize threads
# c_schutzlaser.start()
c_schutzlaser_pi.start()
c_trajektorie_pi.start()