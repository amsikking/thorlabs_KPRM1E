import time
import serial

class Controller:
    '''
    Basic device adaptor for thorlabs KPRM1E Ø1" motorized precision rotation
    stage with DC servo motor driver. Test code runs and seems robust.
    '''
    def __init__(
        self, which_port, name='KPRM1E', verbose=True, very_verbose=False):
        self.name = name
        self.verbose = verbose
        self.very_verbose = very_verbose
        if self.verbose: print("%s: opening..."%self.name, end='')
        try:
            self.port = serial.Serial(
                port=which_port, baudrate=115200, timeout=1)
        except serial.serialutil.SerialException:
            raise IOError(
                '%s: no connection on port %s'%(self.name, which_port))
        if self.verbose: print(" done.")
        self._get_info()
        assert self.model_number == 'KDC101\x00\x00'
        assert self.firmware_v == 131592
        self.EncCnt_per_deg = 1919.6418578623391
        self.EncCnt_per_deg_per_s = 42941.66
        self.EncCnt_per_deg_per_s2 = 14.66
        self._move_tol_deg = 0.01 # move tolerance in degrees
        self._long_timeout_s = 40 # need a lot of time for 360deg move...
        self._set_enable(True)
        self._get_homed_status()
        if not self._homed:
            self._home()
        self.get_position_deg()
        self._set_velocity_parameters(25, 25)
        self._moving = False

    def _send(self, cmd, response_bytes=None):
        if self.very_verbose: print('%s: sending cmd ='%self.name, cmd)
        self.port.write(cmd)
        if response_bytes is not None:
            response = self.port.read(response_bytes)
        else:
            response = None
        assert self.port.inWaiting() == 0
        if self.very_verbose: print('%s: -> response = '%self.name, response)
        return response

    def _get_info(self): # MGMSG_HW_REQ_INFO
        if self.verbose:
            print('%s: getting info'%self.name)
        cmd = b'\x05\x00\x00\x00\x50\x01'
        response = self._send(cmd, response_bytes=90)
        self.model_number = response[10:18].decode('ascii')
        self.type = int.from_bytes(response[18:20], byteorder='little')
        self.serial_number = int.from_bytes(response[6:10], byteorder='little')
        self.firmware_v = int.from_bytes(response[20:24], byteorder='little')
        self.hardware_v = int.from_bytes(response[84:86], byteorder='little')
        if self.verbose:
            print('%s: -> model number  = %s'%(self.name, self.model_number))
            print('%s: -> type          = %s'%(self.name, self.type))
            print('%s: -> serial number = %s'%(self.name, self.serial_number))
            print('%s: -> firmware version = %s'%(self.name, self.firmware_v))
            print('%s: -> hardware version = %s'%(self.name, self.hardware_v))
        return response

    def _get_enable(self): # MGMSG_MOD_REQ_CHANENABLESTATE
        if self.verbose:
            print('%s: getting enable'%self.name)
        cmd = b'\x11\x02\x00\x00\x50\x01'
        response = self._send(cmd, response_bytes=6)
        assert int(response[3]) in (1, 2)
        if int(response[3]) == 1: self.enable = True
        if int(response[3]) == 2: self.enable = False
        if self.verbose:
            print('%s: -> enable = %s'%(self.name, self.enable))
        return self.enable

    def _set_enable(self, enable): # MGMSG_MOD_SET_CHANENABLESTATE
        assert enable in (True, False)
        if enable:     cmd = b'\x10\x02\x00\x01\x50\x01'
        if not enable: cmd = b'\x10\x02\x00\x02\x50\x01'
        if self.verbose:
            print('%s: setting enable = %s'%(self.name, enable))
        self._send(cmd)
        assert self._get_enable() == enable
        if self.verbose:
            print('%s: done setting enable'%self.name)
        return None

    def _get_homed_status(self): # MGMSG_MOT_REQ_STATUSBITS
        if self.verbose:
            print('%s: getting homed status...'%self.name)
        cmd = b'\x29\x04\x00\x00\x50\x01'
        status_bits = self._send(cmd, response_bytes=12)[8:]
        self._homed = bool(status_bits[1] & 4) # bit mask 0x00000400 = homed
        if self.verbose:
            print('%s: -> homed = %s'%(self.name, self._homed))
        return self._homed

    def _home(self, polling_wait_s=0.1): # MGMSG_MOT_MOVE_HOME
        if self.verbose:
            print('%s: homing stage...'%self.name)
        cmd = b'\x43\x04\x00\x00\x50\x01'
        # response_bytes=6 is not documented, discovered by trial and error
        # when the 6 bytes return it seems like the home routine is finished!
        timeout = self.port.timeout
        self.port.timeout = self._long_timeout_s
        self._send(cmd, response_bytes=6)
        self.port.timeout = timeout
        # the controller confirms finished when it's not finished, so this
        # block is to ensure the move is finished and the position is correct:
        while True:
            if self.verbose: print('.', end='')
            time.sleep(polling_wait_s)
            verbose = self.verbose
            self.verbose = False
            self.get_position_deg()
            self.verbose = verbose
            if round(self.position_deg, 2) == 0:
                if self.verbose: print('.')
                break
        if self.verbose:
            print('%s: -> done homing stage'%self.name)
        return None

    def _get_velocity_parameters(self): # MGMSG_MOT_GET_VELPARAMS
        if self.verbose:
            print('%s: getting velocity parameters'%self.name)
        cmd = b'\x14\x04\x00\x00\x50\x01'
        response = self._send(cmd, response_bytes=20)
        self.max_velocity = int.from_bytes( # deg/s
            response[16:20], byteorder='little') / self.EncCnt_per_deg_per_s
        self.acceleration = int.from_bytes( # deg/s^2
            response[12:16], byteorder='little') / self.EncCnt_per_deg_per_s2
        if self.verbose:
            print('%s: -> max velocity = %s'%(self.name, self.max_velocity))
            print('%s: -> acceleration = %s'%(self.name, self.acceleration))
        return self.max_velocity, self.acceleration

    def _set_velocity_parameters(self, max_velocity, acceleration):
        if self.verbose:
            print('%s: setting velocity parameters:'%self.name)
            print('%s: -> max velocity = %s'%(self.name, max_velocity))
            print('%s: -> acceleration = %s'%(self.name, acceleration))
        assert 0 <= max_velocity <= 25
        assert 0 <= acceleration <= 25
        max_velocity_counts = int(round(
            max_velocity * self.EncCnt_per_deg_per_s))
        acceleration_counts = int(round(
            acceleration * self.EncCnt_per_deg_per_s2))
        # MGMSG_MOT_SET_VELPARAMS
        d = bytes([b'\x50'[0] | b'\x80'[0]]) # 'destination byte'
        min_velocity_counts = 0
        m = min_velocity_counts.to_bytes(4, byteorder='little', signed=True)
        v = max_velocity_counts.to_bytes(4, byteorder='little', signed=True)
        a = acceleration_counts.to_bytes(4, byteorder='little', signed=True)
        cmd = (b'\x13\x04\x0E\x00' + d + b'\x01' + self.ch_id_bytes + m + a + v)
        self._send(cmd)
        self._get_velocity_parameters()
        assert round(self.max_velocity, 1) == max_velocity
        assert round(self.acceleration, 1) == acceleration
        if self.verbose:
            print('%s: done setting velocity parameters'%self.name)
        return None

    def identify(self): # MGMSG_MOD_IDENTIFY
        if self.verbose:
            print('%s: -> flashing front panel LEDs'%self.name)
        cmd = b'\x23\x02\x00\x00\x50\x01'
        self._send(cmd)
        return None

    def get_position_deg(self): # MGMSG_MOT_REQ_POSCOUNTER
        if self.verbose:
            print('%s: getting position'%self.name)
        cmd = b'\x11\x04\x00\x00\x50\x01'
        response = self._send(cmd, response_bytes=12)
        self.ch_id_bytes = response[6:8]
        self.position_counts = int.from_bytes(
            response[8:12], byteorder='little')
        self.position_deg = self.position_counts / self.EncCnt_per_deg
        if self.verbose:
            print('%s: -> position = %0.4fdeg'%(self.name, self.position_deg))
        return self.position_deg

    def _finish_move(self, polling_wait_s=0.1):
        if not self._moving: return
        timeout = self.port.timeout
        self.port.timeout = self._long_timeout_s
        self.port.read(20) # collect bytes to confirm move (could parse too...)
        self.port.timeout = timeout
        # the controller confirms finished when it's not finished, so this
        # block is to ensure the move is finished and the position is correct:
        while True:
            if self.verbose: print('.', end='')
            time.sleep(polling_wait_s)
            verbose = self.verbose
            self.verbose = False
            self.get_position_deg()
            self.verbose = verbose
            target, tol = self._target_position_deg, self._move_tol_deg
            if target - tol <= self.position_deg <= target + tol:
                if self.verbose: print('.')
                break
        self._moving = False
        if self.verbose:
            print('%s: -> finished moving'%(self.name))
        return None

    def move_deg(self, move_deg, relative=True, block=True):
        if self._moving: self._finish_move()
        if self.verbose:
            print('%s: moving = %0.4fdeg (relative=%s)'%(
                self.name, move_deg, relative))
        if relative: move_deg = self.position_deg + move_deg
        assert 0 <= move_deg <= 359.99
        position_counts = int(round(move_deg * self.EncCnt_per_deg)) # integer
        # MGMSG_MOT_MOVE_ABSOLUTE
        d = bytes([b'\x50'[0] | b'\x80'[0]]) # 'destination byte'
        p = position_counts.to_bytes(4, byteorder='little', signed=True)
        cmd = (b'\x53\x04\x06\x00' + d + b'\x01' + self.ch_id_bytes + p)
        self._send(cmd)
        self._moving = True
        self._target_position_deg = move_deg
        if block:
            self._finish_move()
        return None

    def close(self):
        if self.verbose: print("%s: closing..."%self.name, end=' ')
        self.port.close()
        if self.verbose: print("done.")
        return None

if __name__ == '__main__':
    mount = Controller('COM11', verbose=True, very_verbose=False)

##    mount.identify()

    print('\n# Get position:')
    mount.get_position_deg()

    print('\n# Test range:')
    mount.move_deg(0, relative=False)
    mount.move_deg(359.99, relative=False)
    mount.move_deg(0, relative=False)

    print('\n# Some relative moves:')
    for moves in range(3):
        move = mount.move_deg(10)
    for moves in range(3):
        move = mount.move_deg(-10)

    print('\n# Non-blocking move:')
    mount.move_deg(10, relative=False, block=False)
    mount.move_deg( 0, relative=False, block=False)
    print('(immediate follow up call forces finish on pending move)')
    print('doing something else')
    mount._finish_move()

    mount.close()
