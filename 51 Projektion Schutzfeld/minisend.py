import paramiko
import tkinter as tk
from tkinter import filedialog

# Auswahl Konzept
user_input = input(" a: run a test file in Innok \n r: Read from RaspPi \n w: Write to RaspPi \n w_i: Write to Innok \n run: run a file in innok \n Bitte wählen Sie: \n ")

if user_input == "a":
    # Verbindung zum Roboter herstellen
    sshRos = paramiko.SSHClient()
    sshRos.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    sshRos.connect('10.21.20.10', port=22, username='heros', password='heros')
    print("Connected to robot")

    # # Führen Sie den Code auf dem Roboter aus, um den Wert zu erhalten und an den Raspberry Pi zu senden
    # stdin, stdout, stderr = sshRos.exec_command('cd Dokumente \n cd test_glenn \n python '
    #     'AVCommunication.py',
    #     get_pty=True)
    # Führen Sie den Code auf dem Roboter aus, um den Wert zu erhalten und an den Raspberry Pi zu senden
    stdin, stdout, stderr = sshRos.exec_command('source /opt/ros/noetic/setup.bash \ncd catkin_ws \n cd src \n cd fixroute \n cd scripts \n cd demo \n python '
        'AVCommunication.py',
        get_pty=True)
    
if user_input == "r":
    ############################## NEW PI (RaspPi4) #########################################

    # Verbindung zum Roboter herstellen
    sshRasp = paramiko.SSHClient()
    sshRasp.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    sshRasp.connect('10.21.20.15', port=22, username='ubuntu', password='heros_ubuntu')
    print("Connected to raspPi")

    # Copy the contents of AV.py to the current workspace
    sftp = sshRasp.open_sftp()
    file_name = input("Please enter the file name to read: ")
        
    # open a GUI dialog for choosing a directory
    root = tk.Tk()
    local_folder_path = filedialog.askdirectory()

    file_path_rasp = '/home/ubuntu/Desktop/Communication/' + file_name
    file_path_local = local_folder_path +'/' + file_name

    sftp.get(file_path_rasp, file_path_local)
    sftp.close()

    print(file_name + " copied to " + local_folder_path + " successfully")

    ############################## NEW PI (RaspPi4) #########################################

    ############################## OLD PI (SophieRasp) #########################################

    # # Verbindung zum Roboter herstellen
    # sshRasp = paramiko.SSHClient()
    # sshRasp.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    # sshRasp.connect('SophieRasp', port=22, username='heros', password='Sophie98')
    # print("Connected to raspPi")

    # # Copy the contents of AV.py to the current workspace
    # sftp = sshRasp.open_sftp()
    # file_name = input("Please enter the file name to read: ")
        
    # # open a GUI dialog for choosing a directory
    # root = tk.Tk()
    # local_folder_path = filedialog.askdirectory()

    # file_path_rasp = '/home/heros/Desktop/Communication/' + file_name
    # file_path_local = local_folder_path +'/' + file_name

    # sftp.get(file_path_rasp, file_path_local)
    # sftp.close()

    # print(file_name + " copied to " + local_folder_path + " successfully")

    ############################## OLD PI (SophieRasp) #########################################

if user_input == "w":
    # create SSH client
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # connect to remote Raspberry Pi
    ssh.connect('10.21.20.15', port=22, username='ubuntu', password='heros_ubuntu')
    print("Connected to raspPi")
    
    # create SFTP client
    sftp = ssh.open_sftp()

    # open a GUI dialog for choosing a directory
    root = tk.Tk()
    local_path = filedialog.askopenfilename()

    # get file name from local path
    file_name = local_path.split('/')[-1]

    # copy file from local machine to remote Raspberry Pi
    remote_path = '/home/ubuntu/Desktop/Communication/' + file_name
    sftp.put(local_path, remote_path)

    # close SFTP client and SSH client
    sftp.close()
    ssh.close()

    print("File copied successfully")

if user_input == "w_i":
    # create SSH client
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # Verbindung zum Roboter herstellen
    ssh.connect('10.21.20.10', port=22, username='heros', password='heros')
    print("Connected to robot")
    
    # create SFTP client
    sftp = ssh.open_sftp()

    # open a GUI dialog for choosing a directory
    root = tk.Tk()
    local_path = filedialog.askopenfilename()

    # get file name from local path
    file_name = local_path.split('/')[-1]

    # copy file from local machine to remote Raspberry Pi
    remote_path = '/home/heros/Dokumente/test_glenn/' + file_name
    sftp.put(local_path, remote_path)

    # close SFTP client and SSH client
    sftp.close()
    ssh.close()

    print("File copied successfully")

if user_input == "run":
    # create SSH client
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # Verbindung zum Roboter herstellen
    ssh.connect('10.21.20.10', port=22, username='heros', password='heros')
    print("Connected to robot")
    
    # create SFTP client
    sftp = ssh.open_sftp()

    # open a GUI dialog for choosing a directory
    root = tk.Tk()
    local_path = filedialog.askopenfilename()

    # get file name from local path
    file_name = local_path.split('/')[-1]

    # copy file from local machine to remote Raspberry Pi
    remote_path = '/home/heros/Dokumente/test_glenn/' + file_name
    sftp.put(local_path, remote_path)

    # close SFTP client and SSH client
    sftp.close()
    ssh.close()

    print("File copied successfully")