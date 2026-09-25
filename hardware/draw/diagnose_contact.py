"""Three camera-monitored stamps using corrected signed load feedback."""
import signal
import time

from draw_reference import Arm, DEFAULT_OUT


def main():
    def interrupt(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")
    signal.signal(signal.SIGTERM, interrupt)
    arm = Arm(DEFAULT_OUT)
    try:
        pose = arm.setup()
        arm.lift(pose["shoulder_lift"])
        anchor = None
        arm.move("shoulder_pan", 12)
        arm.photograph("corrected_contact_before")
        for i in range(3):
            arm.move("shoulder_pan", 12 + 0.84 * i)
            hit = arm.stamp(anchor)
            anchor = hit
            print(f"CONTACT TEST {i + 1}: hit={hit:.3f}, lifted={arm.position('shoulder_lift'):.3f}", flush=True)
            time.sleep(0.5)
            arm.photograph(f"corrected_contact_after_{i + 1}")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
