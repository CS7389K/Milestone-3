import os
import rclpy
import signal

from .cli import CLI


def main():
    rclpy.init()
    cli = CLI()

    def _sigint_handler(signum, frame):
        cli.shutdown()
        rclpy.shutdown()
        os._exit(0)

    signal.signal(signal.SIGINT, _sigint_handler)
    cli.start()
    

if __name__ == "__main__":
    main()