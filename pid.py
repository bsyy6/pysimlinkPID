class PID:
    def __init__(self, Kp, Ki, Kd, dt ,pid_out_max, pid_out_min, N):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.pid_out_max = pid_out_max
        self.pid_out_min = pid_out_min
        self.dt = dt
        self.N = N
        self.Kidt = self.Ki * self.dt
        self.Kddt = self.Kd * self.dt
        self.KdN = self.Kd * self.N
        self.filter_pid_der = 0
        self.integrate = True
        self.pid_int = 0
        self.KbBack = 0 
        self.Kbdt = self.KbBack * self.dt

    def sign(x):
        # returns 1 if x is positive, -1 if x is negative, and 0 if x is 0
        return bool(x > 0) - bool(x < 0)
    
    def calculate(self, PV, SP):
        self.error = SP - PV

        # in case of derivative on measurement
        self.dPV = PV - self.PV_last
        self.PV_last = PV

        # integral
        if self.integrate:
            self.pid_int += self.Ki * self.error * self.dt
            
        # derivative
        self.pid_der = (self.Kd * self.error - self.filter_pid_der) * self.N
        self.filter_pid_der += self.dt * self.pid_der
        
        # proportional
        self.pid_pro = self.Kp * self.error

        out = self.pid_pro  + self.pid_int + self.pid_der  # Note: pid_err is not defined in the given code

        if out >= self.pid_out_max:
            self.KbBack = (self.pid_out_max - out) * self.Kbdt
            out = self.pid_out_max

        if out <= self.pid_out_min:
            self.KbBack = (self.pid_out_min - out) * self.Kbdt
            out = self.pid_out_min

        # anti windup logic
        self.pidSaturated = (out >= self.pid_out_max) or (out <= self.pid_out_min)
        self.pid_int_going_overlimit = (self.sign(out) == self.sign(self.error)) and (self.pidSaturated)

        self.integrate = not (self.pid_int_going_overlimit)

        return out
    
    def getContributions (self):
        return self.pid_pro, self.pid_int, self.pid_der
    
    
