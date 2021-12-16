from math import atan2, asin, pi
from ast import literal_eval
from numpy import sign, zeros

def rads2rpm(rads):
    return (rads * 60) / (2 * pi)


def rpm2rads(rpm):
    return (rpm / 60) * 2 * pi


def PID_fun(Signal, TarValue, ActValue, time_step, Kp, Ki, Kd, I_limit = False):

    error = TarValue - ActValue

    Signal[1] += error * time_step

    if I_limit is not False and abs(Signal[1]) > abs(I_limit):
        Signal[1] = I_limit * sign(Signal[1])

    dervative = (error - Signal[2]) / time_step

    Signal[0] = (Kp * error) + (Ki * Signal[1]) + (Kd * dervative)
    Signal[2] = error
    # return [output, Signal[1], error]


def simple_filter(PrefilterSignal, Signal, alfa):
    # print(PrefilterSignal, Signal,alfa)
    return Signal * alfa + (1 - alfa) * PrefilterSignal


def interpol(ref_value, x_list, y_list):

    if len(x_list) != len(y_list):
        print("lenght of the list don't match")
        return False

    if (x_list[1] - x_list[0]) < 0:
        print("X list has to be rising")
        return False
    n = 0

    if ref_value < x_list[n]:  # Value is out of range
        return y_list[n]

    elif ref_value > x_list[-1]:  # Value is out of range
        return y_list[-1]

    for x in x_list:
        if ref_value == x:
            return y_list[n]

        if ref_value < x:
            y = y_list[n] + (y_list[n - 1] - y_list[n]) * ((ref_value - x_list[n]) / (x_list[n - 1] - x_list[n]))
            return y

        n += 1


def position_speed_profile_dir(ActuPosition, TargetPosition, ControlSpeed, TargetSpeed, TargetAcc, Direction, time_step, TargetDecc = 0):

    Distance = abs(TargetPosition - ActuPosition)
    Direction_inside = sign(TargetPosition - ActuPosition)

    if TargetSpeed == 0 or TargetAcc == 0:
        return 0

    if TargetDecc == 0:
        TargetDecc = TargetAcc

    if sign(ControlSpeed) != Direction and ControlSpeed != 0: # If speed is wrong way, accelerate
        return ControlSpeed + Direction * TargetDecc * time_step

    elif (Distance > 0.5 * TargetDecc * (ControlSpeed / TargetDecc) ** 2 and Direction == Direction_inside):  # is there more distance than breaking requires
        if abs(ControlSpeed) < abs(TargetSpeed):  # Acceleration
            return ControlSpeed + Direction * TargetAcc * time_step
            #  print("kiihtyvyys")
        else:  # Drive
            return Direction * TargetSpeed
            # print("ajo")
    else:  # Breaking
        return ControlSpeed - Direction * TargetDecc * time_step
        # print("jarrutus")


def position_speed_profile(ActuPosition, TargetPosition, ControlSpeed, TargetSpeed, TargetAcc, TargetDecc, time_step):

    Distance = abs(TargetPosition - ActuPosition)
    Direction = sign(TargetPosition - ActuPosition)

    if TargetDecc == 0 or TargetSpeed == 0 or TargetAcc == 0:
        return 0

    if sign(ControlSpeed) != Direction and ControlSpeed != 0: # If speed is wrong way, accelerate
        return ControlSpeed + Direction * TargetDecc * time_step

    if (Distance > 0.5 * TargetDecc * (ControlSpeed / TargetDecc) ** 2):  # is there more distance than breaking requires
        if abs(ControlSpeed) < abs(TargetSpeed):
            return ControlSpeed + Direction * TargetAcc * time_step
            # print("kiihtyvyys")
        else:
            return Direction * TargetSpeed
            # print("ajo")
    else:
        return ControlSpeed - Direction * TargetDecc * time_step
        # print("jarrutus")


def limit_val(min_value, max_value, signal):

    if signal > max_value:
        return max_value

    elif signal < min_value:
        return min_value

    else:
        return signal


def signal_ramp(Target_signal, Control_signal, accelration, deceleration = 0):
    if deceleration == 0:
        deceleration = accelration

    if sign(Target_signal) != sign(Control_signal) and Control_signal != 0:  # If speed is different direction than given, then first brake and then accelrat
        Target_signal = 0

    singal_d = abs(Target_signal) - abs(Control_signal)
    if singal_d < 0 and abs(singal_d) > deceleration:  # Breaking
        # print("jarrutus")
        if Target_signal == 0:
            return Control_signal - deceleration * sign(Control_signal)
        else:
            return Control_signal - deceleration * sign(Target_signal)
    elif singal_d > 0 and abs(singal_d) > accelration:  # Accelrating
        return Control_signal + accelration * sign(Target_signal)
    else:
        return Target_signal

        # print("jarrutus")


class PID:
  # https://www.scilab.org/discrete-time-pid-controller-implementation
    def __init__(self, Kp, Ki, Kd, Ts, N, Max = False, Min = False):
        self.ku_array = zeros(3)
        self.ke_array = zeros(3)
        self.u = zeros(3)  # output vector
        self.e = zeros(3)  # input vector

        self.ku_array[0] = (-(2 + N * Ts) / (1 + N * Ts))  # u1
        self.ku_array[1] = (1 / (1 + N * Ts))  # u2

        self.ke_array[0] = ((Kp * (1 + N * Ts) + Ki * Ts * (1 + N * Ts) + Kd * N) / (1 + N * Ts))  # e0
        self.ke_array[1] = (-(Kp * (2 + N * Ts) + Ki * Ts + 2 * Kd * N) / (1 + N * Ts))  # e1
        self.ke_array[2] = ((Kp + Kd * N) / (1 + N * Ts))  # e2

        self.max = Max
        self.min = Min

    def update(self, ref_sig, act_sig):
        self.u[2] = self.u[1]
        self.u[1] = self.u[0]

        self.e[2] = self.e[1]
        self.e[1] = self.e[0]
        self.e[0] = ref_sig - act_sig

        self.u[0] = -self.ku_array[0] * self.u[1] - self.ku_array[1] * self.u[2] + self.ke_array[0] * self.e[0] + self.ke_array[1] * self.e[1] + self.ke_array[2] * self.e[2]

        if self.max is not False and self.u[0] > self.max:
            self.u[0] = self.max
        elif self.min is not False and self.u[0] < self.min:
            self.u[0] = self.min
        return self.u[0]

    def reset(self):
        self.u = zeros(3)  # output vector
        self.e = zeros(3)  # input vector


def signal_ramp_smooth(Guid_signal, Signal):

    # print(Guid_signal, Signal.value, Signal.speed_value)
    if sign(Guid_signal) != sign(Signal.value) and Signal.value != 0:  # If speed is different direction than given, then first brake and then accelrat
        Guid_signal = 0

    singal_d = abs(Guid_signal) - abs(Signal.value)
    Speed_pre = Signal.speed_value
    distance = 0.5 * Signal.dcc * (abs(Speed_pre) / Signal.dcc) ** 2
    # signal direction
    if Guid_signal == 0:
        dire = sign(Signal.value)
    else:
        dire = sign(Guid_signal)
    if abs(singal_d) < Signal.dcc:
        Signal.value = Guid_signal
        Signal.speed_value = 0
    elif singal_d < 0:  # Breaking
        # print("jarrutus")
        dire *= -1
        if abs(singal_d) <= abs(distance):  # Slow down
            # print("1")
            Signal.speed_value -= Signal.dcc * dire
            if sign(Signal.speed_value) != dire:
                Signal.speed_value = 0
        elif Speed_pre < Signal.brake or sign(Speed_pre) != dire:
            # print("2")
            Signal.speed_value += Signal.acc * dire
            if abs(Signal.speed_value) > Signal.brake:
                Signal.speed_value = Signal.brake * dire
        elif Speed_pre > Signal.brake:
            # print("3")
            Signal.speed_value -= Signal.acc * dire
            if abs(Signal.speed_value) < Signal.brake:
                Signal.speed_value = Signal.brake * dire
        Signal.value += Signal.speed_value
    elif singal_d > 0:  # Accelrating
        # print("kiihdtys")
        if abs(singal_d) <= abs(distance):
            # print("1")
            Signal.speed_value -= Signal.dcc * dire
            if sign(Signal.speed_value) != dire:
                Signal.speed_value = 0
        elif Speed_pre < Signal.drive or sign(Speed_pre) != dire:
            # print("2")
            Signal.speed_value += Signal.acc * dire
            if abs(Signal.speed_value) > Signal.drive and sign(Signal.speed_value) == sign(Signal.value):
                Signal.speed_value = Signal.drive * dire
        elif Speed_pre > Signal.drive:
            # print("3")
            Signal.speed_value -= Signal.acc# * dire
            if abs(Signal.speed_value) < Signal.drive and sign(Signal.speed_value) == sign(Signal.value):
                Signal.speed_value = Signal.drive * dire
        Signal.value += Signal.speed_value

    if (singal_d > 0 and (abs(Guid_signal) - abs(Signal.value)) < 0) or (singal_d < 0 and (abs(Guid_signal) - abs(Signal.value)) > 0):# and Guid_signal != 0):
        Signal.value = Guid_signal
        Signal.speed_value = 0


def signal_ramp_c(Guid_signal, Signal):

    if sign(Guid_signal) != sign(Signal.value) and Signal.value != 0:  # If speed is different direction than given, then first brake and then accelrat
        Guid_signal = 0

    singal_d = abs(Guid_signal) - abs(Signal.value)
    if singal_d < 0 and abs(singal_d) > Signal.brake:  # Breaking
        # print("jarrutus")
        if Guid_signal == 0:
            Signal.value -= Signal.brake * sign(Signal.value)
        else:
            Signal.value -= Signal.brake * sign(Guid_signal)
    elif singal_d > 0 and abs(singal_d) > Signal.drive:  # Accelrating
        Signal.value += Signal.drive * sign(Guid_signal)
    else:
        Signal.value = Guid_signal


class signal:

    def __init__(self, speed, brake, acc = False, dcc = False):
        self.value = 0
        self.drive = speed
        self.brake = brake
        self.speed_value = 0
        if acc is not False:
            self.acc = acc
            if dcc is not False:
                self.dcc = dcc
            else:
                self.dcc = acc
        else:
            self.dcc = 1
            self.acc = 1

    def reset(self):
        self.speed_value = 0
        self.value = 0


def EulerParam2Angles(ep0, ep1, ep2, ep3):
    phi   = atan2(2 * (ep0 * ep1 + ep2 * ep3), 1 - 2 * ((ep1**2) + (ep2**2)))
    if abs(2 * (ep0 * ep2 - ep3 * ep1)) > 1:
        theta = asin(1 * sign(2 * (ep0 * ep2 - ep3 * ep1)))
    else:
        theta = asin(2 * (ep0 * ep2 - ep3 * ep1))
    psi   = atan2(2 * (ep0 * ep3 + ep1 * ep2), 1 - 2 * ((ep2**2) + (ep3**2)))
    # print ("phi: ", phi, ", theta: ", theta, ", psi: ", psi)
    return [phi, theta, psi]


def signal_1D_dir(darket, live_value):

    if live_value > darket:
        return -1
    else:
        return 1

#  Change mevea 3D vector to tuple list
def mev32tuple(object):
    return (object.x, object.y, object.z)


def SpringDamper(x, dx, k, c):
    return k * x + c * dx


def step(g, g0, h0, g1, h1):
    # STEP function
    # Input value: g
    # Output value: h
    # g = x-axis, h = y-axis
    # S-shape is in between g0 and g1, transition is from h0 to h1

    dh = h1 - h0
    dg = (g-g0)/(g1-g0)
    
    if g <= g0:
        return h0
    elif g < g1:
        return h0 + dh * dg**2 *( 3 - 2*dg )
    else:
        return h1

def get_positon(file, tapaus, select):
    #  File where to read
    #  tapaus is what set we are selecting
    #  select is the machine that is selected
    with open(file, 'r') as infile:
        lines = infile.readlines()
        for line in lines:
            pos_set = ast.literal_eval(line)
            if pos_set[0] == tapaus:
                return pos_set[select]
        return 0
