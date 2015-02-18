This is the third version of the code in an attempt to get three Bluetooth boards to connect. I've
made it so that when a board gets connected (or rather is about to print to the screen that it got 
connected) a blue LED lights up. If it doesn't light up that doesn't necessarily mean that the board
isn't connected. What does is that the client prints to the screen something like "problem starting
connection with server 2". From the links you'll discover that for BLE, there can only be one master,
but multiple slaves. In our case the client is the master and is trying to connect to multiple slaves
in the Make_connection() function. Also the BLE status error is being printed to the screen indicating
why the second connection won't be made. The error codes are found in ble_status.h. The current error
code says that the create_connection function is disallowed (I can't figure out why). I think our best
bet at this point is to put the issue to a forum or something.