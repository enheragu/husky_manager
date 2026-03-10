# Husky Web Manager

Web interface for monitoring and managing the Husky ROS system.

## Sudoers Setup (required for service restart)

To allow the web interface to restart services without a password prompt:

```bash
# Copy the sudoers configuration file
sudo cp src/web_manager/sudoers.d/husky-manager /etc/sudoers.d/husky-manager

# Set correct permissions (IMPORTANT)
sudo chmod 440 /etc/sudoers.d/husky-manager

# Verify correct syntax
sudo visudo -c
```

> **Note**: If the user running the web manager is not `administrator`, edit `/etc/sudoers.d/husky-manager` and replace `administrator` with the correct username.

## Usage

The web manager starts automatically via `husky_web_manager.service`, or manually:

```bash
rosrun husky_manager web_manager_node.py
```

Access the interface at: `http://<HUSKY_IP>:5050/manager`
