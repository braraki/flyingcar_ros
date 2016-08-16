#crazyflie_driver
##flyingcar_ros

###config

Config directory has `.yaml` files for each Crazyflie you want to run. Format is:

	```
	$ cat example.yaml
	
	uri: radio://0/80/250K/0xE7E7E7D4
	roll_trim: 4.0
	pitch_trim: 6.0s
	tf_prefix: "example"
	```