# COLA2_LOG package

This package contains nodes related to the logging/storing cola2 parameters and data .

- bag_node: provides services to run and stop the bag.launch file to record ROS bags.
- computer_logger: publishes cpu and RAM usage data and temperature from the vehicle computer.
- default_param_handler: provides services to store current parameters as defaults writting them to the corresponding .YAML files.
- param_logger: publishes all parameters in a topic for logging/debugging purposes.

Dependencies:

The default_param_handler node requires the python module ruamel.yaml. To install it:
	pip install ruamel.yaml

