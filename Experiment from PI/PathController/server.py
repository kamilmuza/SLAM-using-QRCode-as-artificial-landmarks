try:
    import Queue
except ImportError:
    import queue as Queue

import socket


class Server:
    def __init__(self, port, motion):
        self.motion = motion
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', port))
        self.queue = Queue.Queue()
        self.address = None
        self.index = 0

    def process_path(self, data):
        track_data = data.split(',')

        x_planning = []
        y_planning = []
        z_planning = []

        for i, string in enumerate(track_data[3:]):
            if i % 3 == 0:
                x_planning.append(float(string))
            elif i % 3 == 1:
                y_planning.append(float(string))
            else:
                z_planning.append(float(string))

        track = {'constant_t': float(track_data[0]),
                 'constant_k': float(track_data[1]),
                 'cubic': True,
                 'x_planning': x_planning,
                 'y_planning': y_planning,
                 'z_planning': z_planning}

        if self.motion.finished:
            self.motion.experiment_init(track_data[2], False, track)
            self.queue.put('path begin')
        else:
            self.queue.put('experiment running')

    def process_points(self, data):
        track_data = data.split(',')

        x_planning = []
        y_planning = []
        t_planning = []

        for i, string in enumerate(track_data[1:]):
            if i % 3 == 0:
                x_planning.append(float(string))
            elif i % 3 == 1:
                y_planning.append(float(string))
            else:
                t_planning.append(float(string))

        track = {'x_planning': x_planning,
                 'y_planning': y_planning,
                 't_planning': t_planning}
        if self.motion.finished:
            self.motion.experiment_init(track_data[0], False, track)
            self.queue.put('points begin')
        else:
            self.queue.put('experiment running')

    def process_reference(self, data):
        track_data = data.split(',')

        x_planning = []
        y_planning = []
        t_planning = []

        for i, string in enumerate(track_data[1:]):
            if i % 3 == 0:
                x_planning.append(float(string))
            elif i % 3 == 1:
                y_planning.append(float(string))
            else:
                t_planning.append(float(string))

        track = {'x_planning': x_planning,
                 'y_planning': y_planning,
                 't_planning': t_planning}
        if self.motion.finished:
            self.motion.experiment_init(track_data[0], True, track)
            self.queue.put('reference begin')
        else:
            self.queue.put('experiment running')

    def process_parameter(self, data):
        parameter, value = data.split(',')
        if parameter == 'constant_kc' or parameter == 'constant_ki' or parameter == 'constant_kd' or \
                        parameter == 'constant_k1' or parameter == 'constant_k2' or parameter == 'constant_b':
            self.motion.set_parameter(parameter, value)
            self.queue.put('parameter ok')
        else:
            self.queue.put('parameter bad')

    def process_status(self):
        if self.motion.finished:
            self.motion.ask_status()
        else:
            self.queue.put('status %d,%d,%f,%f,%f' % (self.motion.encoder1, self.motion.encoder2,
                                                      self.motion.battery, -1, -1))

    def process_position(self, data):
        if data == 'ask':
            self.queue.put('position %f,%f,%f' % (
                self.motion.globalPositionX, self.motion.globalPositionY, self.motion.globalPositionZ))
        elif data == 'reset':
            self.motion.globalPositionX = 0
            self.motion.globalPositionY = 0
            self.motion.globalPositionZ = 0
            self.queue.put('position %f,%f,%f' % (
                self.motion.globalPositionX, self.motion.globalPositionY, self.motion.globalPositionZ))
        elif data.split()[0] == 'set':
            _, data = data.split()
            pos = data.split(',')
            self.motion.globalPositionX = float(pos[0])
            self.motion.globalPositionY = float(pos[1])
            self.motion.globalPositionZ = float(pos[2])
        else:
            position_data = data.split(',')
            self.motion.movement_init(float(position_data[0]), float(position_data[1]), float(position_data[2]))

    def process_key(self, press, key):
        if press == '1':
           #self.queue.put((self.motion.globalPositionX, self.motion.globalPositionY, self.motion.globalPositionZ))
            self.motion.keys.append(int(key))
        else:
            #self.queue.put((self.motion.globalPositionX, self.motion.globalPositionY, self.motion.globalPositionZ))

            self.motion.keys.remove(int(key))
    def process_request(self, request):

        command, data = request.split(' ', 1)
        print("< %s: %s\n" % (command, data))

        if command == 'path':
            self.process_path(data)

        elif command == 'points':
            self.process_points(data)

        elif command == 'reference':
            self.process_reference(data)

        elif command == 'experiment' and data == 'stop':
            self.motion.finished = True
            self.queue.put('stop ok')

        elif command == 'position':
            self.process_position(data)

        elif command == 'status' and data == 'ask':
            self.process_status()

        elif command == 'slam':
            if data == 'start':
                self.motion.slamflag = True
                self.motion.start_thread_slam()
            else:
                self.motion.slamflag = False

        elif command == 'parameter':
            self.process_parameter(data)

        elif command == 'localization':
            pass

        elif command == 'movement':
            parameter, value = data.split(',')
            self.motion.continuous(parameter, float(value))

        elif command == 'key':
            press, key = data.split(',')
            self.process_key(press, key)

        elif command == 'compass':
            if data == 'ask':
                self.motion.update_compass_angle()
                self.queue.put('compass %f' % (self.motion.compass_angle))
            else:
                self.motion.calibrate_compass()
        elif command == 'rotate':
            self.motion.rotate_inplace(eval(data))

        elif command == 'odometry':
            self.motion.odometryTuner(eval(data))

        else:
            print
            'bad message'
            self.queue.put('bad message')

    def sender_thread(self):
        while 1:
            if self.address:
                if not self.queue.empty():
                    to_send = self.queue.get()
                    self.socket.sendto(to_send, self.address)
                    print(">> %s" % to_send)
                if self.index != self.motion.index:
                    to_send = self.motion.list[self.index]
                    self.socket.sendto(to_send, self.address)
                    print("> %s" % to_send)
                    self.index += 1
                    if self.index >= 2000:
                        self.index = 0

    def run(self):
        while 1:
            request, address = self.socket.recvfrom(1024)
            self.address = address
            self.process_request(request)
            # print 'Request:[%s]' % request
            #self.socket.sendto(request, address)

            #todo: add continuos traslation backward
            #todo: add continuos rotation
            #todo: add fixed rotation