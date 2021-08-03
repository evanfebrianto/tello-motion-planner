class PID():
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

        self.last_cte = 0.0

    def updateError(self, cte):
        self.p_error = cte
        self.i_error += cte
        self.d_error = cte - self.last_cte

        self.last_cte = self.p_error

    def totalError(self):
        return -self.p_error*self.Kp - self.i_error*self.Ki - self.d_error*self.Kd;