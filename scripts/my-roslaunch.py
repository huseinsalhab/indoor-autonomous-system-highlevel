import roslaunch
from time import sleep 

package = 'ians_control'
executable = 'ians_control_server'
executable = 'keyboard_listener.py'
executable = 'motor_controller.py'
server_node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(server_node)
print(process.is_alive)
sleep(30)
process.stop()
