
class Initial:
    def __init__(self):
        self.time_to_hit = [[0,40,50],[40,0,60],[50,60,0]]
        self.time_interval = 100
        init_plan = [0] * self.time_interval
        ac = Aircraft(0, 0, init_plan)
        ac1 = ac.set_ac('ac1', 0, 0, init_plan)
        ac2 = ac.set_ac('ac2', 0, 0, init_plan)
        ac3 = ac.set_ac('ac3', 0, 0, init_plan)
        self.all_ac = {**ac1, **ac2, **ac3}


    def get_tth(self):
        return self.time_to_hit

    def get_acs(self):
        return self.all_ac

    def get_intvl(self):
        return self.time_interval

class Aircraft:
    def __init__(self, height, velocity, trvl_plan):
        self.height = height
        self.velocity = velocity

    def set_ac(self, name, height, velocity, trvl_plan):
        self.name = name
        self.height = height
        self.velocity = velocity
        self.trvl_plan = trvl_plan
        ac = dict()
        ac[name] = [height, velocity, trvl_plan]
        return ac
