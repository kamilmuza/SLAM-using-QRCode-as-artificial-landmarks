import time
import server
import thread
import controller
motion = controller.Controller()

my_server = server.Server(50007, motion)
thread.start_new_thread(my_server.run, ())
thread.start_new_thread(my_server.sender_thread(), ())

while 1:
    time.sleep(2)
