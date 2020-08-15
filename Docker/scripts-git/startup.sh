# Allow SSH logins for root TODO: don't use root
echo PermitRootLogin yes >> /etc/ssh/sshd_config
# Assign a password to root
echo root:r | chpasswd
# Start SSH service
service ssh restart

# Setup catkin_ws
/scripts/catkin_setup.sh

# Append anything needed to bashrc, including ros sources
/scripts/append_bashrc.sh

# Print the hostname
echo '[42mContainer started successfully.[0m'
echo Container ID: $(cat /etc/hostname)

# Keep container running 
tail -f /dev/null