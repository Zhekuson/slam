import threading
import time


class SlamThread(threading.Thread):
    def __init__(self, job, verbose):
        super().__init__()
        self.job = job
        self.result = None
        self.verbose = verbose

    
    def run(self):
        start_time = time.time()
        if self.verbose:
            print("Startedf execution for " + self.job.name + " at" + str(start_time))

        self.result = self.job.func()

        if self.verbose:
            print("Execution take " + str(time.time() - start_time) + " " + self.job.name)


class Job:
    def __init__(self, func, name):
        self.func = func
        self.name = name 


class ParallelExecutor:
    def __init__(self, verbose = True):
        self.verbose = verbose

    def execute(self, jobs):
        threads = [SlamThread(job, self.verbose) for job in jobs]

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()

        return [thread.result for thread in threads]
