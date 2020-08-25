$help = @"
Usage: .\win-make COMMAND

A docker package for running ROS and related applications through your browser. 

Commands:
  start      Start the docker container
  stop       Stop the docker container
  fix        Fix any issues introduced by force-stop
"@

if ($args[0] -eq "start") {
    # Create volume with rosperistent folder.
    $p = (Get-Location) -replace ' ', '` '
    Invoke-Expression "docker volume create --opt type=none --opt device=${p}\rospersistent --opt o=bind rospersistent"
    Invoke-Expression "docker volume inspect rospersistent"

    # Start container.
    $cmd = "docker run -d --rm -v rospersistent:/my_ros_data:consistent -p 80:8080 -p 6080:6080 --name tb3-ros cosi119/tb3-ros:latest"
    Invoke-Expression $cmd 
} elseif ($args[0] -eq "stop") {
    # Stop container.
    Invoke-Expression "docker kill tb3-ros"
} elseif ($args[0] -eq "fix") {
    Invoke-Expression "docker network prune -f"
	Invoke-Expression "docker container prune -f"
} else {
    Write-Output $help
}
