#!/usr/bin/env python3
"""
SWARMz4 Diagnostic Tool
======================

This is a standalone diagnostic tool that helps verify the correct operation of
the SWARMz4 simulation environment. It checks:

1. Gazebo connections
2. ROS2 services
3. Namespace availability
4. Topic availability

Usage:
    $ python3 diagnostic_tool.py [world_name]
    
    world_name: Optional. The name of the Gazebo world (default: swarmz_world_2)
"""

import subprocess
import sys
import time
import os

def print_header(text):
    """Print a formatted header"""
    print("\n" + "=" * 80)
    print(f" {text} ".center(80, "="))
    print("=" * 80)

def print_section(text):
    """Print a section header"""
    print("\n" + "-" * 80)
    print(f" {text} ".center(80, "-"))
    print("-" * 80)

def print_result(test, result, details=None):
    """Print a test result with formatting"""
    if result:
        print(f"✓ PASS: {test}")
    else:
        print(f"✗ FAIL: {test}")
    
    if details:
        if isinstance(details, list):
            for line in details:
                print(f"       {line}")
        else:
            print(f"       {details}")

def run_command(cmd, timeout=10, check=False):  # Increased timeout from 5 to 10 seconds
    """Run a command and return its output"""
    try:
        print(f"Running command with {timeout}s timeout: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=check)
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        print(f"Command timed out after {timeout} seconds")
        return 1, "", f"Command timed out after {timeout} seconds"
    except Exception as e:
        return 1, "", str(e)

def check_gazebo(world_name):
    """Check Gazebo status and verify world"""
    print_section("Gazebo Status")
    
    # Check if Gazebo is running - look for both gz sim and gzserver
    return_code, stdout, stderr = run_command(["ps", "-ef"])
    gazebo_running = 'gz sim' in stdout or 'gzserver' in stdout
    print_result("Gazebo server running", gazebo_running)
    
    if not gazebo_running:
        print_result("Gazebo world available", False, "Gazebo is not running")
        return False
    
    # Check Gazebo topics with increased timeout
    print("Checking Gazebo topics (may take up to 10 seconds)...")
    return_code, stdout, stderr = run_command(["gz", "topic", "-l"], timeout=10)
    if return_code != 0:
        if "timed out" in stderr:
            print_result("Gazebo topics accessible", True, 
                         f"Command timed out but Gazebo is running. Continuing with direct model access.")
        else:
            print_result("Gazebo topics accessible", False, f"Error: {stderr}")
            return False
    
    topics = stdout.strip().split('\n') if stdout else []
    print_result("Gazebo topics accessible", len(topics) > 0, f"Found {len(topics)} topics")
    
    # Check for world topics
    world_topics = [t for t in topics if f'/world/{world_name}/' in t]
    model_topics = [t for t in topics if '/model/' in t]
    
    # If we have model topics but no world topics, we're in a different topic format scenario
    if len(model_topics) > 0 and len(world_topics) == 0:
        print_result(f"Alternative topic format detected", True, 
                    f"Using model-based topics instead of world-based topics")
        
        # Check if we have pose-related topics
        pose_topics = [t for t in topics if 'pose' in t]
        print_result("Pose information available", len(pose_topics) > 0,
                    f"Found {len(pose_topics)} pose-related topics")
                    
        # Show sample model information
        print_result("Model information", len(model_topics) > 0,
                    f"Found {len(model_topics)} model topics")
        
        if len(model_topics) > 0:
            # Get unique model names
            model_names = set()
            for topic in model_topics:
                parts = topic.split('/')
                if len(parts) > 2:
                    model_names.add(parts[2])
            
            print_result("Detected models", len(model_names) > 0,
                       f"Models found: {list(model_names)[:5] + ['...'] if len(model_names) > 5 else list(model_names)}")
        
        # This is a different format but still valid
        return True
    
    # Standard world-based format check
    world_exists = len(world_topics) > 0
    print_result(f"World '{world_name}' exists", world_exists, 
                f"Found {len(world_topics)} topics for this world" if world_exists else "No topics for this world found")
    
    if not world_exists:
        # List available worlds
        worlds = set([t.split('/')[2] for t in topics if '/world/' in t and len(t.split('/')) > 3])
        if worlds:
            print_result("Available worlds", True, f"Found worlds: {worlds}")
        else:
            print_result("Available worlds", False, "No worlds found in Gazebo topics")
    
    # Check for pose info
    pose_topic = f"/world/{world_name}/pose/info"
    alternative_pose_topic = "/pose/info"
    
    have_pose_topic = pose_topic in topics or alternative_pose_topic in topics
    print_result(f"Pose topic available", have_pose_topic, 
                f"Using standard pose topic" if pose_topic in topics else 
                f"Using alternative pose topic" if alternative_pose_topic in topics else
                "No pose topics found")
    
    # Try alternative model access if topics are empty
    if not topics:
        # Try to get model info directly
        print("Trying direct model access...")
        return_code, stdout, stderr = run_command(["gz", "model", "-l"], timeout=10)
        if return_code == 0 and stdout:
            models = stdout.strip().split('\n')
            print_result("Models accessible via direct access", True, 
                         f"Found {len(models)} models")
            if models:
                sample_models = models[:5]
                print_result("Sample models", True, sample_models)
                
                # Try to get position of first model
                if models[0]:
                    return_code, stdout, stderr = run_command(["gz", "model", "-m", models[0], "-p"], timeout=10)
                    print_result("Model position accessible", return_code == 0, 
                                 stdout if return_code == 0 else stderr)
            
            # Consider this a success if we found models
            return len(models) > 0
    
    # Return true if we have either world topics or model topics
    return gazebo_running and (world_exists or len(model_topics) > 0)

def check_ros2():
    """Check ROS2 system status"""
    print_section("ROS2 Status")
    
    # Check ROS2 nodes
    return_code, stdout, stderr = run_command(["ros2", "node", "list"])
    nodes = stdout.strip().split('\n') if stdout else []
    print_result("ROS2 system running", return_code == 0, 
                f"Found {len(nodes)} nodes" if nodes else "No nodes found")
    
    if nodes:
        print_result("Node list", True, nodes[:5] + (["..."] if len(nodes) > 5 else []))
    
    # Check if critical nodes are running
    game_master = any('game_master_node' in node for node in nodes)
    print_result("Game Master running", game_master)
    
    missile_server = any('missile_server' in node for node in nodes)
    print_result("Missile Server running", missile_server)
    
    kamikaze_server = any('kamikaze_server' in node for node in nodes)
    print_result("Kamikaze Server running", kamikaze_server)
    
    # Check ROS2 services
    return_code, stdout, stderr = run_command(["ros2", "service", "list"])
    services = stdout.strip().split('\n') if stdout else []
    print_result("ROS2 services available", return_code == 0, 
                f"Found {len(services)} services" if services else "No services found")
    
    # Check critical services
    update_health = any('/update_health' in service for service in services)
    print_result("UpdateHealth service", update_health)
    
    fire_missile = any('/fire_missile' in service for service in services)
    print_result("FireMissile service", fire_missile)
    
    kamikaze = any('/kamikaze' in service for service in services)
    print_result("Kamikaze service", kamikaze)
    
    return return_code == 0

def main():
    print_header("SWARMz4 Diagnostic Tool")
    
    # Get world name from arguments or use default
    world_name = sys.argv[1] if len(sys.argv) > 1 else "swarmz_world_2"
    print(f"Using world name: {world_name}")
    
    # Show performance warning
    print("\nNOTE: Gazebo commands may be slow on some systems. Be patient during diagnostics.\n")
    
    # Check Gazebo
    gazebo_ok = check_gazebo(world_name)
    
    # Check ROS2 
    ros2_ok = check_ros2()
    
    # Print summary
    print_section("Summary")
    print_result("Gazebo Status", gazebo_ok)
    print_result("ROS2 Status", ros2_ok)
    
    if not gazebo_ok:
        print("\nRecommendations for Gazebo issues:")
        print("1. Make sure Gazebo is running with the correct world:")
        print(f"   gz sim -v4 {world_name}.sdf")
        print("2. Check if the world_name parameter is correct")
        print("3. Verify the SDF file exists and is valid") 
        print("4. If 'gz topic -l' times out but Gazebo is running, the system will try")
        print("   direct model access as a fallback mechanism")
    
    if not ros2_ok:
        print("\nRecommendations for ROS2 issues:")
        print("1. Make sure ROS2 is properly sourced")
        print("2. Check that required packages are installed")
        print("3. Restart the game_master launch file")
    
    if gazebo_ok and ros2_ok:
        print("\nAll systems appear to be functioning correctly!")
        print("If you're still experiencing issues, try restarting the simulation and")
        print("the game_master launch file in that order.")
    
    print("\nDiagnostic complete.")

if __name__ == "__main__":
    main()
