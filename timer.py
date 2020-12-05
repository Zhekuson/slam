from threading import Thread
import time


class TimerThread(Thread):
    def __init__(self, func, interval):
        super().__init__()
        self.func = func
        self.interval = interval


    def run(self):
        time.sleep(self.interval)



class Timer:
    def __init__(self, func, interval):
        self.thread = TimerThread(func, interval)

    def stop(self):
        self.thread._stop()

    def start(self):
        self.thread.start()
