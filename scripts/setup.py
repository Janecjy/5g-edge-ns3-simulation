#!/usr/bin/env python3
"""
Setup script for 5G Edge NS-3 Simulation
Reads remote node configuration from YAML and executes setup commands on RAN and Edge nodes.
"""

import yaml
import subprocess
import sys
import argparse
from pathlib import Path
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RemoteNodeSetup:
    def __init__(self, config_file):
        """Initialize with configuration file path."""
        self.config_file = Path(config_file)
        self.config = self.load_config()
        
    def load_config(self):
        """Load configuration from YAML file."""
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            logger.info(f"Configuration loaded from {self.config_file}")
            return config
        except FileNotFoundError:
            logger.error(f"Configuration file {self.config_file} not found")
            sys.exit(1)
        except yaml.YAMLError as e:
            logger.error(f"Error parsing YAML file: {e}")
            sys.exit(1)
    
    def execute_remote_command(self, hostname, username, command, timeout=300):
        """Execute command on remote node via SSH."""
        ssh_command = [
            'ssh',
            '-o', 'StrictHostKeyChecking=no',
            '-o', 'UserKnownHostsFile=/dev/null',
            f'{username}@{hostname}',
            command
        ]
        
        logger.info(f"Executing on {hostname}: {command}")
        
        try:
            result = subprocess.run(
                ssh_command,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            
            if result.returncode == 0:
                logger.info(f"Command succeeded on {hostname}")
                if result.stdout:
                    logger.info(f"Output: {result.stdout.strip()}")
            else:
                logger.error(f"Command failed on {hostname} with exit code {result.returncode}")
                if result.stderr:
                    logger.error(f"Error: {result.stderr.strip()}")
                return False
                
        except subprocess.TimeoutExpired:
            logger.error(f"Command timed out on {hostname}")
            return False
        except Exception as e:
            logger.error(f"Error executing command on {hostname}: {e}")
            return False
            
        return True
    
    def setup_ran_node(self):
        """Setup RAN node with required dependencies and NS-3 simulation."""
        if 'ran' not in self.config:
            logger.error("RAN configuration not found in config file")
            return False
            
        ran_config = self.config['ran']
        hostname = ran_config.get('hostname')
        username = ran_config.get('username', 'janechen')
        
        if not hostname:
            logger.error("RAN hostname not specified in config")
            return False
            
        logger.info(f"Setting up RAN node: {username}@{hostname}")
        
        # Commands to execute on RAN node
        ran_commands = [
            # Update system packages
            "sudo apt update",
            
            # Change ownership of /mydata/ directory
            f"sudo chown -R {username} /mydata/",
            
            # Install required dependencies
            "sudo apt-get install -y cmake libc6-dev sqlite3 libsqlite3-dev libeigen3-dev",
            
            # Install mahimahi dependencies
            "sudo apt install -y protobuf-compiler libprotobuf-dev autotools-dev dh-autoreconf iptables pkg-config dnsmasq-base apache2-bin debhelper libssl-dev ssl-cert libxcb-present-dev libcairo2-dev libpango1.0-dev apache2-dev git",
            
            # Install mahimahi from source
            "cd /tmp && if [ ! -d 'mahimahi' ]; then git clone https://github.com/ravinet/mahimahi; fi",
            "cd /tmp/mahimahi && ./autogen.sh",
            "cd /tmp/mahimahi && ./configure",
            "cd /tmp/mahimahi && make",
            "cd /tmp/mahimahi && sudo make install",
            
            # Clone the repository (if not exists)
            "cd /mydata && if [ ! -d '5g-edge-ns3-simulation' ]; then git clone git@github.com:Janecjy/5g-edge-ns3-simulation.git; fi",
            
            # Setup the project
            "cd /mydata/5g-edge-ns3-simulation && git checkout 5g-ran-dev",
            "cd /mydata/5g-edge-ns3-simulation && git submodule update --init --recursive",
            "cd /mydata/5g-edge-ns3-simulation && ./ns3 configure --enable-examples --enable-tests",
            
            # Build the project
            "cd /mydata/5g-edge-ns3-simulation && ./ns3 build",
            
            # Run the simulation
            "cd /mydata/5g-edge-ns3-simulation && ./ns3 run cttc-nr-demo",
        ]
        
        # Execute commands sequentially
        for command in ran_commands:
            if not self.execute_remote_command(hostname, username, command):
                logger.error(f"Failed to execute command on RAN node: {command}")
                return False
                
        logger.info("RAN node setup completed successfully")
        return True
    
    def setup_edge_node(self):
        """Setup Edge node (if configuration exists)."""
        if 'edge' not in self.config:
            logger.info("Edge configuration not found, skipping edge setup")
            return True
            
        edge_config = self.config['edge']
        hostname = edge_config.get('hostname')
        username = edge_config.get('username', 'janechen')
        
        if not hostname:
            logger.error("Edge hostname not specified in config")
            return False
            
        logger.info(f"Setting up Edge node: {username}@{hostname}")
        
        # Add edge-specific setup commands here
        edge_commands = [
            "sudo apt update",
            f"sudo chown -R {username} /mydata/",
            
            # Install mahimahi dependencies
            "sudo apt install -y protobuf-compiler libprotobuf-dev autotools-dev dh-autoreconf iptables pkg-config dnsmasq-base apache2-bin debhelper libssl-dev ssl-cert libxcb-present-dev libcairo2-dev libpango1.0-dev apache2-dev git",
            
            # Install mahimahi from source
            "cd /tmp && if [ ! -d 'mahimahi' ]; then git clone https://github.com/ravinet/mahimahi; fi",
            "cd /tmp/mahimahi && ./autogen.sh",
            "cd /tmp/mahimahi && ./configure",
            "cd /tmp/mahimahi && make",
            "cd /tmp/mahimahi && sudo make install",
            
            # Add more edge-specific commands as needed
        ]
        
        for command in edge_commands:
            if not self.execute_remote_command(hostname, username, command):
                logger.error(f"Failed to execute command on Edge node: {command}")
                return False
                
        logger.info("Edge node setup completed successfully")
        return True
    
    def setup_all(self):
        """Setup all nodes."""
        success = True
        
        if not self.setup_ran_node():
            success = False
            
        if not self.setup_edge_node():
            success = False
            
        return success

def main():
    parser = argparse.ArgumentParser(description='Setup 5G Edge NS-3 Simulation on remote nodes')
    parser.add_argument('--config', '-c', default='config.yaml', 
                       help='Path to YAML configuration file (default: config.yaml)')
    
    args = parser.parse_args()
    
    if not Path(args.config).exists():
        logger.error(f"Configuration file {args.config} not found")
        sys.exit(1)
    
    setup = RemoteNodeSetup(args.config)
    
    # Setup all nodes
    if not setup.setup_all():
        logger.error("Setup failed")
        sys.exit(1)
    
    logger.info("Setup completed successfully")

if __name__ == "__main__":
    main()
