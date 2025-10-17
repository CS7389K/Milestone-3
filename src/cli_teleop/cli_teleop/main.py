import sys
import time
import signal

from .controller import TeleopController
from .util.cli import print_menu, handle_key
from .errors import ValidationError, BackendError


def main():
    ctrl = TeleopController()

    # graceful Ctrl+C
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

    while True:
        print_menu(ctrl)
        sys.stdout.write("Enter command: ")
        sys.stdout.flush()

        key = sys.stdin.readline().strip().lower()  # simple, line-buffered input
        # If you want non-blocking, switch to curses or tty raw mode.

        try:
            handle_key(ctrl, key)
        except ValidationError as ve:
            print(f"[ValidationError] {ve}\n")
            time.sleep(0.1)  # tiny delay to avoid spam
        except BackendError as be:
            # Backend failed (e.g., ROS publisher not ready)
            print(f"[BackendError] {be}. Issuing STOP for safety.\n")
            try:
                ctrl.stop()
            except Exception:
                pass
        except SystemExit:
            print("Quitting...")
            # Stop robot for safety on exit
            try:
                ctrl.stop()
            except Exception:
                pass
            break
        except Exception as e:
            # Catch-all to prevent runaway robot on unexpected error
            print(f"[UnexpectedError] {e}. Issuing STOP for safety.\n")
            try:
                ctrl.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()
