from KukaEnv import KukaEnv
import time

def run():
    kuka = KukaEnv(gifsave=0)
    pos,vel,torques = kuka.joint_states()
    pl, vl, tl = kuka.joint_limits()

    while(True):
        kuka.step(1)
        time.sleep(1.0/240.0)




if __name__=="__main__":
    run()
