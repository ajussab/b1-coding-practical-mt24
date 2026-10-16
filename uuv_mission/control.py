# PD controller for the submarine's depth
# Every time step it will compare the actual depth with the target depth 
# and calculate the error and then work out how much force to provide
# (how much to push the submarine up or down) to reduce the error

class PDController:

    # SETUP: this runs once, when the controller is created
    def __init__(self, kp=0.15, kd=0.6):
        self.kp = kp # proportional gain
        self.kd = kd # derivative gain
        self.previous_error = 0 #store the previous error (memory of
        # e[t-1])

    # CONTROL: this runs every time step, when the controller is called
    # takes target depth r(t) and actual depth y(t), returns action u(t)
    def __call__(self, reference, observation):
        error = reference - observation # e[t] = r[t] - y[t]
        change_in_error = error - self.previous_error # e[t] - e[t-1]
        action = self.kp * error + self.kd * change_in_error # u[t] = Kp*e[t] + Kd*(e[t]-e[t-1])
        self.previous_error = error # remember e[t] for next step
        return action # send u[t] back to the submarine to push it up or down

    # RESET: wipes the memory so a new simulation starts fresh
    def reset(self): 
        self.previous_error = 0