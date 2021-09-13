'''

Description: Catching Keyboard Interrupt exception

Use time library to simulate a long-running task


'''

from signal import signal, SIGINT
from sys import exit 

def handler(signal_received, frame):
    # handle clean up actions here
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)


if __name__ == '__main__':
    # register the signal handler
    # tells python to run the handler() function when SIGINT is received
    signal(SIGINT, handler)

    print('Running, press ctrl-c to exit')
    while True:
        # do nothing and use cpu resources until SIGINT received
        pass
