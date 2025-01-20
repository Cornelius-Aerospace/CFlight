import serial, threading

class gscs_interface():
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.connection = serial.Serial(port, baudrate)
        self.is_connected = False
        self.gscs_status = None
        self.espnow_status = None
        self.lora_status = None
        self.fc_paired = False
        self.fc_status = None

        self.data_buffer = None
        self.new_data = False
        self.last_message = None
        self.on_message_callbacks = []

        self.update_thread = None
    
    def connect(self):
        self.connection = serial.Serial(self.port, self.baudrate)
        while self.connection.readline() != b'GSCS: Ready\n':
            pass
        self.is_connected = True
        self.update_thread = threading.Thread(target=self.update)
        return True
    
    def disconnect(self):
        self.is_connected = False
        self.connection.close()
        return True
    
    def update(self):
        while self.is_connected:
            self.data_buffer = self.connection.readline().decode('utf-8')
            print(self.data_buffer)
            self.new_data = self.decode_message()
            if self.new_data:     
                for callback in self.on_message_callbacks:
                    callback(self.last_message)
    
    def decode_message(self):
        # TODO
        return False